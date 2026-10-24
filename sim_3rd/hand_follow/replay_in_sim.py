"""Feed a recorded human command stream into the repo's dynamic hand model and measure following.

  python replay_in_sim.py --repo C:/path/robot_hand --recording recordings/hand_YYYYMMDD_HHMMSS.csv

The recording holds the commands the live app produced (after calibration, filter and joint
limits) with the time each was issued. They are applied as a zero-order hold, exactly as a
servo bus would deliver them, to sim/remote_drive_v1 run_simulation (motor, PWM, cable,
controller and link dynamics as the repo models them; uncalibrated, not hardware).

Reported per joint:
  lag_ms     time shift that best aligns the simulated joint with its command (cross-correlation,
             0..250 ms), only for joints that actually moved (> 5 deg range)
  max_err    max |simulated - command| in degrees
  filter_lag_ms / truth lag  only for synthetic recordings that carry the true angles
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from hand_follow.retarget import NAMES  # noqa: E402


def best_lag(ref, sig, dt, max_ms=250):
    ref = ref - ref.mean(); sig = sig - sig.mean()
    best, lag = -np.inf, 0
    for k in range(0, int(max_ms / 1000 / dt) + 1):
        c = np.dot(ref[:len(ref) - k], sig[k:]) / max(1, len(ref) - k)
        if c > best:
            best, lag = c, k
    return lag * dt * 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", type=Path, required=True)
    ap.add_argument("--recording", type=Path, required=True)
    ap.add_argument("--variant", default="remote20", choices=["remote20", "remote16", "baseline20"])
    ap.add_argument("--dt", type=float, default=0.000125)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    sys.path.insert(0, str(a.repo / "sim" / "remote_drive_v1"))
    from model import build_model, run_simulation

    rows = list(csv.reader(open(a.recording, encoding="utf-8")))
    head, data = rows[0], rows[1:]
    col = {h: i for i, h in enumerate(head)}
    t_cmd = np.array([float(r[col["t_command_s"]]) for r in data])
    t_cap = np.array([float(r[col["t_capture_s"]]) for r in data])
    cmd = np.array([[float(r[col["cmd_" + n]]) for n in NAMES] for r in data])
    truth = None
    if "truth_" + NAMES[0] in col:
        truth = np.array([[float(r[col["truth_" + n]]) for n in NAMES] for r in data])

    t0 = t_cmd[0]
    time = np.arange(0, t_cmd[-1] - t0 + 0.3, a.dt)
    idx = np.clip(np.searchsorted(t_cmd - t0, time, side="right") - 1, 0, len(t_cmd) - 1)
    commands = np.radians(cmd[idx])                       # zero-order hold, as a bus delivers
    system = build_model(a.variant, dict(dt_s=a.dt))
    trace = run_simulation(system, time, commands)
    q = np.degrees(trace["q"])
    c = np.degrees(commands)

    report = dict(recording=str(a.recording), variant=a.variant, dt_s=a.dt, duration_s=float(time[-1]),
                  frames=len(data), command_rate_hz=float(1 / np.median(np.diff(t_cmd))),
                  frame_age_at_command_ms=dict(median=float(np.median(t_cmd - t_cap) * 1000),
                                               p95=float(np.percentile(t_cmd - t_cap, 95) * 1000)),
                  joints={})
    step = max(1, int(0.001 / a.dt))
    for j, n in enumerate(NAMES):
        moved = np.ptp(c[:, j]) > 5
        r = dict(range_deg=float(np.ptp(c[:, j])), max_err_deg=float(np.abs(q[:, j] - c[:, j]).max()))
        if moved:
            r["lag_ms"] = best_lag(c[::step, j], q[::step, j], a.dt * step)
            if truth is not None:
                tt = np.interp(time, t_cap - t0, truth[:, j])
                r["robot_vs_truth_lag_ms"] = best_lag(tt[::step], q[::step, j], a.dt * step)
                r["command_vs_truth_lag_ms"] = best_lag(tt[::step], c[::step, j], a.dt * step)
        report["joints"][n] = r
    lags = [v["lag_ms"] for v in report["joints"].values() if "lag_ms" in v]
    report["robot_lag_ms"] = dict(median=float(np.median(lags)), max=float(np.max(lags))) if lags else None
    out = a.out or a.recording.with_name(a.recording.stem + "_replay")
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        sel = [1, 2, 5, 17, 18, 19]
        fig, axes = plt.subplots(len(sel), 1, figsize=(11, 2.2 * len(sel)), sharex=True)
        for ax, j in zip(axes, sel):
            if truth is not None:
                ax.plot(t_cap - t0, truth[:, j], color="0.6", lw=1, label="true hand (synthetic)")
            ax.plot(time, c[:, j], "k--", lw=1, label="command")
            ax.plot(time, q[:, j], lw=1.2, label="simulated joint")
            ax.set_ylabel(NAMES[j], fontsize=8)
            ax.grid(alpha=.3)
        axes[0].legend(fontsize=7, loc="upper right")
        axes[-1].set_xlabel("time (s)")
        fig.suptitle(f"{a.recording.name} -> {a.variant} (uncalibrated model)")
        fig.tight_layout()
        fig.savefig(out / "replay.png", dpi=120)
    except ImportError:
        pass
    print(json.dumps({k: report[k] for k in ["frames", "command_rate_hz", "frame_age_at_command_ms",
                                              "robot_lag_ms"]}, indent=1))
    print("per joint lag (ms):", {n: round(v["lag_ms"], 1) for n, v in report["joints"].items() if "lag_ms" in v})
    if truth is not None:
        print("filter lag vs truth (ms):", {n: round(v["command_vs_truth_lag_ms"], 1)
                                            for n, v in report["joints"].items() if "command_vs_truth_lag_ms" in v})
    print("report:", out / "report.json")


if __name__ == "__main__":
    main()
