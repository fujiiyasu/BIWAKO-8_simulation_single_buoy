import os
import pandas as pd
import matplotlib.pyplot as plt

# フォルダ設定
input_dir = "csv"
output_dir = "graph"
os.makedirs(output_dir, exist_ok=True)

# 各CSVファイルに対して処理
for filename in os.listdir(input_dir):
    if filename.endswith(".csv"):
        input_path = os.path.join(input_dir, filename)
        df = pd.read_csv(input_path)

        # 必須カラムのチェック
        if "gps_lat" not in df.columns or "gps_lon" not in df.columns:
            print(f"⚠️ スキップ: {filename}（gps_lat または gps_lon が見つかりません）")
            continue

        # 軌跡プロット（GPSノイズなし）
        plt.figure(figsize=(6, 6))
        plt.plot(df["gps_lon"], df["gps_lat"], color="darkorange", label="True Trajectory (No Noise)")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title(f"True Trajectory - {filename}")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()

        # 保存
        output_path = os.path.join(output_dir, filename.replace(".csv", "_true_trajectory.png"))
        plt.savefig(output_path)
        plt.close()

        print(f"✅ GPSノイズなしの軌跡グラフを保存: {output_path}")
