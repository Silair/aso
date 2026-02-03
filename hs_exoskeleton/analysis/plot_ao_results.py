from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt


def load_vector(path: Path) -> np.ndarray:
    data = np.loadtxt(path, ndmin=1)
    return np.asarray(data, dtype=float)


def load_matrix(path: Path, cols: int) -> np.ndarray:
    data = np.loadtxt(path, ndmin=2)
    data = np.asarray(data, dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < cols:
        raise ValueError(f"Expected at least {cols} columns in {path}, got {data.shape[1]}")
    return data[:, :cols]


def find_settle_index(
    omega: np.ndarray,
    target: float,
    dt: float,
    tol_ratio: float = 0.02,
    window_sec: float = 1.0,
) -> Optional[int]:
    tol = abs(target) * tol_ratio
    window = max(1, int(round(window_sec / dt)))
    within = np.abs(omega - target) <= tol
    for i in range(0, len(omega) - window + 1):
        if np.all(within[i : i + window]):
            return i
    return None


def save_figure(fig: plt.Figure, out_dir: Path, name: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / f"{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(out_dir / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="AO result plots and metrics")
    parser.add_argument("--root", type=str, default=".", help="Project root (contains output/)" )
    parser.add_argument("--step-size", type=float, default=0.005, help="Control step size (s)")
    parser.add_argument("--record-stride", type=int, default=5, help="Log stride in steps")
    parser.add_argument("--dt", type=float, default=None, help="Override dt (s)")
    parser.add_argument("--omega-target", type=float, default=4.83, help="Target omega (rad/s)")
    parser.add_argument("--omega-tol", type=float, default=0.02, help="Tolerance ratio for settling")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    output_dir = root / "output"
    ao_path = output_dir / "AO" / "Output" / "ao_log.txt"
    up_path = output_dir / "PIDController" / "Output" / "Up.txt"
    ud_path = output_dir / "PIDController" / "Output" / "Ud.txt"

    if not ao_path.exists():
        raise FileNotFoundError(f"Missing {ao_path}")
    if not up_path.exists():
        raise FileNotFoundError(f"Missing {up_path}")
    if not ud_path.exists():
        raise FileNotFoundError(f"Missing {ud_path}")

    ao_log = load_matrix(ao_path, cols=5)
    up = load_vector(up_path)
    ud = load_vector(ud_path)

    n = min(len(ao_log), len(up), len(ud))
    ao_log = ao_log[:n]
    up = up[:n]
    ud = ud[:n]

    dt = args.dt if args.dt is not None else args.step_size * args.record_stride
    t = np.arange(n) * dt

    ao_input = ao_log[:, 0]
    theta_ref = ao_log[:, 1]
    theta_act = ao_log[:, 2]
    phi = ao_log[:, 3]
    omega = ao_log[:, 4]

    error = theta_ref - theta_act
    rms_error = float(np.sqrt(np.mean(error ** 2)))
    max_error = float(np.max(np.abs(error)))

    settle_idx = find_settle_index(omega, args.omega_target, dt, args.omega_tol)
    if settle_idx is None:
        settle_time = None
        ss_slice = slice(int(0.8 * len(omega)), len(omega))
    else:
        settle_time = settle_idx * dt
        ss_slice = slice(settle_idx, len(omega))

    steady_var = float(np.var(omega[ss_slice]))

    out_fig = output_dir / "figures"

    plt.style.use("seaborn-v0_8")

    fig1, ax1 = plt.subplots(figsize=(9, 4.5))
    ax1.plot(t, ao_input, color="#999999", linewidth=1.0, label="Input (noisy)")
    ax1.plot(t, theta_ref, color="#1f77b4", linewidth=2.0, label="Theta_Ref (AO)")
    ax1.plot(t, theta_act, color="#ff7f0e", linewidth=1.6, label="Theta_Act (motor)")
    ax1.set_title("Gait Synchronization & Filtering")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Angle (rad)")
    ax1.legend(frameon=True)
    ax1.grid(True, alpha=0.3)
    ax1.annotate("AO smooths noisy input",
                 xy=(t[len(t)//3], theta_ref[len(t)//3]),
                 xytext=(t[len(t)//3], theta_ref[len(t)//3] + 0.4),
                 arrowprops=dict(arrowstyle="->", color="#1f77b4"),
                 color="#1f77b4")
    save_figure(fig1, out_fig, "fig1_gait_sync_filtering")

    fig2, ax2 = plt.subplots(figsize=(8.5, 4.5))
    ax2.plot(t, omega, color="#2ca02c", linewidth=1.8, label="Omega")
    ax2.axhline(args.omega_target, color="#d62728", linestyle="--", linewidth=1.2, label="Target")
    if settle_time is not None:
        ax2.axvline(settle_time, color="#9467bd", linestyle=":", linewidth=1.3, label=f"Settling: {settle_time:.2f}s")
        ax2.text(settle_time, args.omega_target, f"  {settle_time:.2f}s", color="#9467bd", va="bottom")
    ax2.set_title("Frequency Adaptation (Omega)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Omega (rad/s)")
    ax2.legend(frameon=True)
    ax2.grid(True, alpha=0.3)
    save_figure(fig2, out_fig, "fig2_frequency_adaptation")

    fig3, (ax3a, ax3b) = plt.subplots(1, 2, figsize=(10.5, 4.2), gridspec_kw={"width_ratios": [2.2, 1]})
    ax3a.plot(t, error, color="#1f77b4", linewidth=1.4)
    ax3a.set_title("Tracking Error (Time)")
    ax3a.set_xlabel("Time (s)")
    ax3a.set_ylabel("Error (rad)")
    ax3a.grid(True, alpha=0.3)
    ax3a.text(0.02, 0.95, f"RMS: {rms_error:.4f} rad\nMax: {max_error:.4f} rad",
              transform=ax3a.transAxes, va="top", ha="left",
              bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    ax3b.hist(error, bins=30, color="#ff7f0e", alpha=0.85)
    ax3b.set_title("Error Distribution")
    ax3b.set_xlabel("Error (rad)")
    ax3b.set_ylabel("Count")
    ax3b.grid(True, alpha=0.3)
    save_figure(fig3, out_fig, "fig3_tracking_error_stats")

    dtheta_ref = np.gradient(theta_ref, dt)
    dtheta_act = np.gradient(theta_act, dt)

    fig4, ax4 = plt.subplots(figsize=(6.2, 6.0))
    ax4.plot(theta_ref, dtheta_ref, color="#1f77b4", linewidth=1.6, label="Reference")
    ax4.plot(theta_act, dtheta_act, color="#ff7f0e", linewidth=1.6, label="Actual")
    ax4.set_title("Phase Portrait / Limit Cycle")
    ax4.set_xlabel(r"$\theta$ (rad)")
    ax4.set_ylabel(r"$\dot{\theta}$ (rad/s)")
    ax4.legend(frameon=True)
    ax4.grid(True, alpha=0.3)
    save_figure(fig4, out_fig, "fig4_phase_portrait")

    fig5, ax5 = plt.subplots(figsize=(8.5, 4.5))
    ax5.plot(t, up, color="#1f77b4", linewidth=1.5, label="Up (P term)")
    ax5.plot(t, ud, color="#d62728", linewidth=1.5, label="Ud (D term)")
    ax5.set_title("Control Effort (PID Contribution)")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Control Output")
    ax5.legend(frameon=True)
    ax5.grid(True, alpha=0.3)
    save_figure(fig5, out_fig, "fig5_control_effort")

    metrics_path = out_fig / "metrics.txt"
    metrics_path.write_text(
        "AO Analysis Metrics\n"
        f"Samples: {n}\n"
        f"dt: {dt}\n"
        f"Omega target: {args.omega_target}\n"
        f"Settling time: {settle_time if settle_time is not None else 'N/A'}\n"
        f"Steady variance: {steady_var}\n"
        f"RMS error: {rms_error}\n"
        f"Max error: {max_error}\n"
    )

    print("[OK] Figures saved to:", out_fig)
    print("[OK] Metrics:")
    print("  RMS error:", rms_error)
    print("  Max error:", max_error)
    print("  Settling time:", settle_time)
    print("  Steady variance:", steady_var)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
