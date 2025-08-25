#!/usr/bin/env python3
import os; os.environ.setdefault("MPLBACKEND", "Agg")
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# 경로 설정
base_dir = Path(__file__).resolve().parent.parent / "data"
cmd_csv = base_dir / "cmd_timeseries.csv"
profile_csv = base_dir / "last_profile.csv"
recalc_csv = base_dir / "recalc_timeseries.csv"

# 출력 디렉토리 (data 폴더)
plot_dir = base_dir
plot_dir.mkdir(parents=True, exist_ok=True)

# 1. 속도 프로파일 (거리 vs vlim, v_forward, v_final)
profile_df = pd.read_csv(profile_csv)
plt.figure(figsize=(10,6))
plt.plot(profile_df["s"], profile_df["vlim"], label="vlim_kappa")
plt.plot(profile_df["s"], profile_df["v_fwd"], label="v_forward")
plt.plot(profile_df["s"], profile_df["v_final"], label="v_final")
plt.xlabel("Distance s [m]")
plt.ylabel("Speed v [m/s]")
plt.title("Speed Profile vs Distance")
plt.grid(True)
plt.legend()
plt.savefig(plot_dir / "v_profile_py.png", dpi=150)
plt.close()

# 2. 곡률 (거리 vs kappa)
plt.figure(figsize=(10,6))
plt.plot(profile_df["s"], profile_df["kappa"], label="curvature kappa")
plt.xlabel("Distance s [m]")
plt.ylabel("Curvature kappa [1/m]")
plt.title("Curvature vs Distance")
plt.grid(True)
plt.savefig(plot_dir / "kappa_py.png", dpi=150)
plt.close()

# 3. Slack (시간 vs min_slack)
recalc_df = pd.read_csv(recalc_csv)
plt.figure(figsize=(10,6))
plt.plot(recalc_df["t"], recalc_df["min_slack"], label="min_slack")
plt.xlabel("Time t [s]")
plt.ylabel("Slack [m/s^2]")
plt.title("Min Slack vs Time")
plt.grid(True)
plt.savefig(plot_dir / "min_slack_py.png", dpi=150)
plt.close()

# 4. 명령 속도 vs 측정 속도 (시간 vs v_cmd, v_meas)
cmd_df = pd.read_csv(cmd_csv)
plt.figure(figsize=(10,6))
plt.plot(cmd_df["t"], cmd_df["v_cmd"], label="v_cmd")
plt.plot(cmd_df["t"], cmd_df["v_meas"], label="v_meas")
plt.xlabel("Time t [s]")
plt.ylabel("Speed v [m/s]")
plt.title("Commanded vs Measured Speed")
plt.legend()
plt.grid(True)
plt.savefig(plot_dir / "v_cmd_meas_py.png", dpi=150)
plt.close()

print(f"Plots saved to: {plot_dir}")
