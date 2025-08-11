import pandas as pd
import matplotlib.pyplot as plt
import os

# CSVファイル読み込み
file_path = "202508071156.csv"
df = pd.read_csv(file_path)

# グラフ保存先ディレクトリ作成
output_dir = "graph"
os.makedirs(output_dir, exist_ok=True)

# ==========================
# 1. Trajectory: ノイズあり＋ノイズなしGPS
# ==========================
plt.figure(figsize=(8, 6))
plt.plot(df["gps_noise_lon"], df["gps_noise_lat"], label="GPS with Noise", linestyle="--", color="orange")
plt.plot(df["gps_lon"], df["gps_lat"], label="GPS (Ground Truth)", linestyle="-", color="blue")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Robot Trajectory")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "trajectory.png"))
plt.close()

# ==========================
# 2. その他のグラフ定義
# ==========================
plots = [
    ("Speed over Time", "time_s", "speed_mps", "speed_time.png"),
    ("Power Consumption over Time", "time_s", "power_W", "power_time.png"),
    ("Thrust vs Speed", "thrust", "speed_mps", "thrust_vs_speed.png"),
    ("Angle Error over Time", "time_s", "angle_deg", "angle_time.png"),
    ("Distance to Target over Time", "time_s", "distance_m", "distance_time.png"),
]

# グラフを生成して保存
for title, x, y, filename in plots:
    plt.figure()
    plt.plot(df[x], df[y], label=f"{y} vs {x}")
    plt.xlabel(x)
    plt.ylabel(y)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename))
    plt.close()

# ==========================
# 3. ヒートマップ: スラスタ指令
# ==========================
plt.figure(figsize=(10, 5))
thruster_df = df[["thruster_dir_1", "thruster_dir_2", "thruster_dir_3", "thruster_dir_4"]]
plt.imshow(thruster_df.T, aspect='auto', cmap='coolwarm', interpolation='nearest')
plt.colorbar(label="Thruster Direction")
plt.yticks(range(4), ["T1", "T2", "T3", "T4"])
plt.xlabel("Timestep")
plt.title("Thruster Directions Heatmap")
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "thruster_heatmap.png"))
plt.close()

# ==========================
# 4. 累積電力グラフ
# ==========================
df["delta_time"] = df["time_s"].diff().fillna(0)
df["energy_J"] = df["power_W"] * df["delta_time"]
df["cumulative_energy_J"] = df["energy_J"].cumsum()
df["cumulative_energy_Wh"] = df["cumulative_energy_J"] / 3600

plt.figure(figsize=(8, 5))
plt.plot(df["time_s"], df["cumulative_energy_Wh"], label="Cumulative Energy (Wh)", color="green")
plt.xlabel("Time [s]")
plt.ylabel("Cumulative Energy [Wh]")
plt.title("Cumulative Energy Consumption over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "cumulative_energy.png"))
plt.close()

# ==========================
# 5. 出力ファイル一覧表示
# ==========================
graph_files = sorted(os.listdir(output_dir))
graph_files
