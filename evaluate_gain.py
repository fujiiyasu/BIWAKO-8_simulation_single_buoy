import os
import re
import math
import glob
import numpy as np
import pandas as pd

# ========= ユーザー設定 =========
LOG_DIR = "./csv"                 # 実験ログCSVが入っているフォルダ
FILE_PATTERN = "*.csv"             # CSVのパターン
WAYPOINT_CSV = "./waypoint/test.csv"   # const.parameter.way_point_file と同じ
NUM_LAPS = 3                       # 評価対象の周回数
OUTPUT_SUMMARY_CSV = "evaluation_summary.csv"
PRINT_TOP_N = 10                   # 終了時に表示する上位件数
# 重み：小さいほど良い指標なので「1 - minmax」で正規化して加点化
WEIGHTS = {
    "crosstrack_rmse_m": 0.10,   # 主指標（軌道誤差RMSE）
    "energy_Wh": 1.0,          # エネルギー
    "total_time_s": 0.20,       # 所要時間
    "control_total_variation": 0.20,  # 制御の滑らかさ（小さいほど良い）
    "heading_rmse_deg": 0.10,   # 方位誤差
}
# =============================


# ---------- ユーティリティ ----------
def load_waypoints(csv_path):
    wps = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            cols = line.split(",")
            if len(cols) < 2:
                continue
            lon = float(cols[0])
            lat = float(cols[1])
            wps.append((lat, lon))  # [lat, lon]
    if len(wps) < 2:
        raise ValueError("ウェイポイントが2点未満です。")
    return np.array(wps)  # shape=(N,2)

def latlon_to_xy(lat, lon, lat0, lon0, R=6371000.0):
    x = (np.deg2rad(lon - lon0)) * math.cos(math.radians(lat0)) * R
    y = (np.deg2rad(lat - lat0)) * R
    return x, y

def polyline_segments(Pxy):
    A = Pxy[:-1, :]  # start
    B = Pxy[1:,  :]  # end
    return A, B

def point_to_polyline_distances(Qxy, A, B):
    # Qxy: (N,2), A/B: (M,2)
    N = Qxy.shape[0]
    Q = Qxy[:, None, :]        # (N,1,2)
    A_ = A[None, :, :]         # (1,M,2)
    B_ = B[None, :, :]         # (1,M,2)
    AB = B_ - A_               # (1,M,2)
    AQ = Q - A_                # (N,M,2)
    AB_len2 = np.sum(AB*AB, axis=2)  # (1,M)
    AB_len2 = np.where(AB_len2 < 1e-12, 1e-12, AB_len2)
    t = np.sum(AQ*AB, axis=2) / AB_len2  # (N,M)
    t = np.clip(t, 0.0, 1.0)
    P = A_ + t[..., None]*AB            # (N,M,2)
    D = Q - P
    dist = np.sqrt(np.sum(D*D, axis=2)) # (N,M)
    return np.min(dist, axis=1)         # (N,)

def detect_first_k_laps(df, k=3):
    """ waypoint_numが0に戻るたびに1ラップ完了。最初のkラップ分の末尾indexを返す """
    if "waypoint_num" not in df.columns:
        return len(df)
    wp = df["waypoint_num"].to_numpy()
    laps = 0
    last_wp = wp[0]
    for i in range(1, len(wp)):
        if wp[i] < last_wp:
            laps += 1
            if laps >= k:
                return i
        last_wp = wp[i]
    return len(wp)

def energy_Wh(df):
    if "cumulative_energy_Wh" in df.columns:
        return float(df["cumulative_energy_Wh"].iloc[-1])
    dt = df["time_s"].diff().fillna(0.0).to_numpy()
    W  = df["power_W"].to_numpy()
    E_J = np.sum(W * dt)
    return float(E_J / 3600.0)

def control_total_variation(df):
    tv_thrust = np.abs(df["thrust"].diff().fillna(0.0)).sum() if "thrust" in df.columns else 0.0
    tv_dirs = 0.0
    for c in ["thruster_dir_1","thruster_dir_2","thruster_dir_3","thruster_dir_4"]:
        if c in df.columns:
            tv_dirs += np.abs(df[c].diff().fillna(0.0)).sum()
    return float(tv_thrust + tv_dirs)

def heading_rmse(df):
    if "angle_deg" not in df.columns:
        return np.nan
    a = df["angle_deg"].to_numpy()
    return float(np.sqrt(np.mean(a*a)))

def total_time(df):
    return float(df["time_s"].iloc[-1] - df["time_s"].iloc[0])

def parse_params_from_filename(fname):
    """
    例: 20250807_153706_Kp0.6_Kd2.5.csv -> (0.6, 2.5)
    .csv の手前で厳密に止める。整数/小数の両方を許可。
    """
    bn = os.path.basename(fname)
    pat = r"Kp([0-9]+(?:\.[0-9]+)?)_Kd([0-9]+(?:\.[0-9]+)?)\.csv$"
    m = re.search(pat, bn)
    if not m:
        # 末尾が .csv でないなど、別名にも耐えるフォールバック
        m = re.search(r"Kp([0-9]+(?:\.[0-9]+)?)_Kd([0-9]+(?:\.[0-9]+)?)", bn)
    kp = float(m.group(1)) if m else float("nan")
    kd = float(m.group(2)) if m else float("nan")
    return kp, kd

# ---------- 1ファイル評価 ----------
def evaluate_file(csv_path, wp_xy, lat0, lon0):
    df = pd.read_csv(csv_path)

    # ラップ切り出し（3周）
    end_idx = detect_first_k_laps(df, k=NUM_LAPS)
    df = df.iloc[:end_idx].copy()

    # ノイズなしGPS軌跡をXYへ（列名: gps_lat/gps_lon）
    lat = df["gps_lat"].to_numpy()
    lon = df["gps_lon"].to_numpy()
    x, y = latlon_to_xy(lat, lon, lat0, lon0)
    traj_xy = np.stack([x, y], axis=1)

    # クロストラック距離（ポリライン最近傍）
    A, B = polyline_segments(wp_xy)
    dists = point_to_polyline_distances(traj_xy, A, B)
    crosstrack_rmse = float(np.sqrt(np.mean(dists**2)))

    # 付帯指標
    E_Wh   = energy_Wh(df)
    T_s    = total_time(df)
    H_rmse = heading_rmse(df)
    TV     = control_total_variation(df)

    kp, kd = parse_params_from_filename(csv_path)

    return {
        "file": os.path.basename(csv_path),
        "Kp": kp,
        "Kd": kd,
        "crosstrack_rmse_m": crosstrack_rmse,
        "energy_Wh": E_Wh,
        "total_time_s": T_s,
        "control_total_variation": TV,
        "heading_rmse_deg": H_rmse,
    }


# ---------- スコアリング＆ランキング ----------
def minmax_01(series):
    vmin = series.min()
    vmax = series.max()
    if not np.isfinite(vmin) or not np.isfinite(vmax) or abs(vmax - vmin) < 1e-12:
        return pd.Series(np.zeros(len(series)), index=series.index)
    return (series - vmin) / (vmax - vmin)

def compute_scores(df, weights):
    # すべて「小さいほど良い」→ 1 - minmax で加点化
    score_cols = {}
    for k, w in weights.items():
        norm = minmax_01(df[k])            # 0..1（大きい=悪い）
        score = (1.0 - norm) * float(w)    # 大きい=良い
        df[f"score_{k}"] = score
        score_cols[f"score_{k}"] = score
    df["total_score"] = sum(score_cols.values())
    return df

def rank_results(df, weights):
    df = compute_scores(df, weights)
    # 合計点降順→主指標昇順などでタイブレーク
    sort_cols = ["total_score", "crosstrack_rmse_m", "energy_Wh", "total_time_s",
                 "control_total_variation", "heading_rmse_deg"]
    ascending = [False, True, True, True, True, True]
    return df.sort_values(sort_cols, ascending=ascending).reset_index(drop=True)


# ---------- メイン ----------
def main():
    # ウェイポイント読み込み→XYへ
    wps = load_waypoints(WAYPOINT_CSV)   # [lat,lon]
    lat0, lon0 = wps[0, 0], wps[0, 1]
    wp_x, wp_y = latlon_to_xy(wps[:,0], wps[:,1], lat0, lon0)
    wp_xy = np.stack([wp_x, wp_y], axis=1)

    # ログ一覧
    paths = sorted(glob.glob(os.path.join(LOG_DIR, FILE_PATTERN)))
    if not paths:
        raise FileNotFoundError(f"ログが見つかりません: {os.path.join(LOG_DIR, FILE_PATTERN)}")

    # 評価
    rows = []
    for p in paths:
        try:
            rows.append(evaluate_file(p, wp_xy, lat0, lon0))
        except Exception as e:
            print(f"[WARN] 評価失敗: {p} ({e})")

    if not rows:
        raise RuntimeError("評価可能なログがありません。")

    res = pd.DataFrame(rows)

    # スコア＆ランキング
    ranked = rank_results(res, WEIGHTS)

    # 保存
    ranked.to_csv(OUTPUT_SUMMARY_CSV, index=False, encoding="utf-8-sig")

    # 画面表示（上位N件）
    print("\n===== Evaluation Result (Top {}) =====".format(min(PRINT_TOP_N, len(ranked))))
    show_cols = [
        "file","Kp","Kd",
        "crosstrack_rmse_m","energy_Wh","total_time_s",
        "control_total_variation","heading_rmse_deg",
        "total_score"
    ]
    print(ranked.loc[:PRINT_TOP_N-1, show_cols].to_string(index=False))

    # ベストの詳細
    best = ranked.iloc[0]
    print("\n=== Best Trial (by total_score with tie-breakers) ===")
    print(f"file: {best['file']}")
    print(f"Kp: {best['Kp']}, Kd: {best['Kd']}")
    print(f"CrossTrack RMSE [m]: {best['crosstrack_rmse_m']:.3f}")
    print(f"Energy [Wh]: {best['energy_Wh']:.3f}")
    print(f"Total Time [s]: {best['total_time_s']:.2f}")
    print(f"Control TV: {best['control_total_variation']:.2f}")
    print(f"Heading RMSE [deg]: {best['heading_rmse_deg']:.2f}")
    print(f"Total Score: {best['total_score']:.3f}")
    print(f"\n=> 全結果は {OUTPUT_SUMMARY_CSV} に保存しました。")

if __name__ == "__main__":
    main()
