import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os
import glob

LOG_DIR = os.path.dirname(os.path.abspath(__file__))
FUSE_LIMIT = 20  # amps — adjust to your fuse rating

csv_files = sorted(glob.glob(os.path.join(LOG_DIR, "motor_current_*.csv")))

for csv_path in csv_files:
    df = pd.read_csv(csv_path)
    if df.empty or len(df) < 2:
        print(f"Skipping {os.path.basename(csv_path)} (too short)")
        continue

    df["elapsed_s"] = df["elapsed_ms"] / 1000.0
    name = os.path.splitext(os.path.basename(csv_path))[0]

    motor_cols  = ["lf_A", "rf_A", "lb_A", "rb_A", "frontIntake_A", "backIntake_A", "lShooter_A", "rShooter_A"]
    hub_cols    = ["controlHub_A", "expansionHub_A"]

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f"Motor Current Log — {name}", fontsize=13, fontweight="bold")
    gs = gridspec.GridSpec(3, 1, hspace=0.45)

    # --- Plot 1: Individual motor currents ---
    ax1 = fig.add_subplot(gs[0])
    for col in motor_cols:
        if col in df.columns:
            ax1.plot(df["elapsed_s"], df[col], label=col.replace("_A", ""), linewidth=1)
    ax1.set_title("Individual Motor Currents")
    ax1.set_ylabel("Current (A)")
    ax1.legend(ncol=4, fontsize=8, loc="upper right")
    ax1.grid(True, alpha=0.3)

    # --- Plot 2: Hub totals ---
    ax2 = fig.add_subplot(gs[1])
    for col in hub_cols:
        if col in df.columns:
            ax2.plot(df["elapsed_s"], df[col], label=col.replace("_A", ""), linewidth=1.2)
    if "hub_total_A" in df.columns:
        ax2.plot(df["elapsed_s"], df["hub_total_A"], label="hub_total", linewidth=1.5, color="black", linestyle="--")
    ax2.axhline(y=FUSE_LIMIT, color="red", linestyle="--", linewidth=1, label=f"Fuse limit ({FUSE_LIMIT}A)")
    ax2.set_title("Hub Currents (includes motors + servos + sensors)")
    ax2.set_ylabel("Current (A)")
    ax2.legend(ncol=4, fontsize=8, loc="upper right")
    ax2.grid(True, alpha=0.3)

    # --- Plot 3: Motor total vs hub total ---
    ax3 = fig.add_subplot(gs[2])
    if "motor_total_A" in df.columns:
        ax3.plot(df["elapsed_s"], df["motor_total_A"], label="motor_total", linewidth=1.5, color="steelblue")
    if "hub_total_A" in df.columns:
        ax3.plot(df["elapsed_s"], df["hub_total_A"], label="hub_total", linewidth=1.5, color="darkorange")
    ax3.axhline(y=FUSE_LIMIT, color="red", linestyle="--", linewidth=1, label=f"Fuse limit ({FUSE_LIMIT}A)")
    ax3.fill_between(df["elapsed_s"], FUSE_LIMIT,
                     df["hub_total_A"] if "hub_total_A" in df.columns else 0,
                     where=(df["hub_total_A"] > FUSE_LIMIT) if "hub_total_A" in df.columns else False,
                     color="red", alpha=0.2, label="Over fuse limit")
    ax3.set_title("Total Current Draw vs Fuse Limit")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Current (A)")
    ax3.legend(ncol=4, fontsize=8, loc="upper right")
    ax3.grid(True, alpha=0.3)

    out_path = os.path.join(LOG_DIR, name + ".png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {out_path}")

print("Done.")
