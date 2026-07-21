#!/usr/bin/env python3
"""Fit the optional gimbal torque feedforward model from a 1 kHz CSV log.

Expected columns:
  axis, angular_velocity_rad_s, angular_acceleration_rad_s2,
  motor_torque_nm, position_rad

`position_rad` is only required for pitch.  Keep torque feedforward disabled on
both computers until the fitted model has been replayed and reviewed.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load_rows(path: Path, axis: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows: list[tuple[float, float, float, float]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            if row.get("axis", "").strip().lower() != axis:
                continue
            rows.append(
                (
                    float(row["angular_acceleration_rad_s2"]),
                    float(row["angular_velocity_rad_s"]),
                    float(row.get("position_rad", 0.0) or 0.0),
                    float(row["motor_torque_nm"]),
                )
            )
    if len(rows) < 100:
        raise ValueError(f"{axis}: need at least 100 samples, found {len(rows)}")
    values = np.asarray(rows, dtype=np.float64)
    return values[:, :3], values[:, 3], np.arange(len(rows))


def design_matrix(state: np.ndarray, axis: str, gravity_zero: float) -> np.ndarray:
    acceleration = state[:, 0]
    velocity = state[:, 1]
    columns = [acceleration, velocity, np.sign(velocity)]
    if axis == "pitch":
        columns.append(np.sin(state[:, 2] - gravity_zero))
    return np.column_stack(columns)


def robust_fit(matrix: np.ndarray, torque: np.ndarray) -> np.ndarray:
    weights = np.ones(torque.shape[0])
    coefficients = np.zeros(matrix.shape[1])
    for _ in range(12):
        root_weight = np.sqrt(weights)
        coefficients, *_ = np.linalg.lstsq(
            matrix * root_weight[:, None], torque * root_weight, rcond=None
        )
        residual = torque - matrix @ coefficients
        scale = 1.4826 * np.median(np.abs(residual - np.median(residual))) + 1.0e-9
        normalized = np.abs(residual) / (1.5 * scale)
        weights = np.where(normalized <= 1.0, 1.0, 1.0 / normalized)
    return coefficients


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--axis", choices=("yaw", "pitch"), required=True)
    parser.add_argument("--pitch-gravity-zero-rad", type=float, default=0.0)
    parser.add_argument("--max-validation-rmse-nm", type=float, default=0.08)
    args = parser.parse_args()

    state, torque, _ = load_rows(args.csv, args.axis)
    split = max(80, int(0.8 * len(torque)))
    train = design_matrix(state[:split], args.axis, args.pitch_gravity_zero_rad)
    validation = design_matrix(state[split:], args.axis, args.pitch_gravity_zero_rad)
    coefficients = robust_fit(train, torque[:split])
    predicted = validation @ coefficients
    rmse = float(np.sqrt(np.mean(np.square(torque[split:] - predicted))))

    names = ["inertia", "viscous", "coulomb"]
    if args.axis == "pitch":
        names.append("gravity")
    print(f"# validation_rmse_nm: {rmse:.6f}")
    print(f"# validation_samples: {len(predicted)}")
    for name, value in zip(names, coefficients):
        print(f"{args.axis}_{name}: {value:.9g}")
    if args.axis == "pitch":
        print(f"pitch_gravity_zero_rad: {args.pitch_gravity_zero_rad:.9g}")
    if rmse > args.max_validation_rmse_nm:
        print("# REJECT: validation RMSE exceeds threshold; keep torque feedforward disabled")
        return 2
    print("# PASS: replay and hardware-limit tests are still required before enabling torque FF")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
