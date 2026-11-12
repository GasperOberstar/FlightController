#!/usr/bin/env python3
"""Plot a rate-loop simulation CSV (t, setpoint, gyro, cmd, rate).

Usage:
    python tools/plot_rate_loop.py sim_output/rate_loop.csv
"""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> int:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "sim_output/rate_loop.csv")
    if not path.exists():
        print(f"file not found: {path}")
        return 1

    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["t"]

    fig, (ax_rate, ax_cmd) = plt.subplots(
        2, 1, sharex=True, figsize=(10, 7),
        gridspec_kw={"height_ratios": [3, 1]},
    )

    ax_rate.plot(t, data["setpoint"], "k--", label="setpoint")
    ax_rate.plot(t, data["rate"], label="true rate")
    ax_rate.plot(t, data["gyro"], label="gyro (filtered)", alpha=0.8)
    ax_rate.set_ylabel("rate [rad/s]")
    ax_rate.legend()
    ax_rate.grid(True)

    ax_cmd.plot(t, data["cmd"], color="tab:red", label="PID output")
    ax_cmd.set_ylabel("cmd [normalized]")
    ax_cmd.set_xlabel("time [s]")
    ax_cmd.legend()
    ax_cmd.grid(True)

    # Quick numbers for the step: overshoot and 10-90% rise time.
    sp_final = data["setpoint"][-1]
    if sp_final > 0:
        rate = data["rate"]
        overshoot = max(0.0, (rate.max() - sp_final) / sp_final * 100.0)
        t_step = t[np.argmax(data["setpoint"] > 0)]
        i10 = np.argmax(rate >= 0.1 * sp_final)
        i90 = np.argmax(rate >= 0.9 * sp_final)
        title = f"overshoot {overshoot:.1f}%"
        if rate.max() >= 0.9 * sp_final:
            title += f", 10-90% rise {1000.0 * (t[i90] - t[i10]):.0f} ms"
        else:
            title += ", never reached 90%"
        ax_rate.set_title(f"{path.name}: {title} (step at {t_step:.3f} s)")

    fig.tight_layout()
    out = path.with_suffix(".png")
    fig.savefig(out, dpi=120)
    print(f"saved {out}")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())