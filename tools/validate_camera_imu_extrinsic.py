#!/usr/bin/env python3
"""Validate the body_from_camera rotation stored in control.yaml."""

from __future__ import annotations

import argparse
from pathlib import Path
import re

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("control_yaml", type=Path)
    parser.add_argument("--max-orthogonality-error", type=float, default=1.0e-3)
    args = parser.parse_args()

    text = args.control_yaml.read_text(encoding="utf-8")
    match = re.search(
        r"body_from_camera\s*:\s*!!opencv-matrix.*?data\s*:\s*\[([^]]+)\]",
        text,
        flags=re.DOTALL,
    )
    if match is None:
        raise SystemExit("body_from_camera OpenCV matrix was not found")
    values = [float(value.strip()) for value in match.group(1).split(",")]
    if len(values) != 9:
        raise SystemExit("body_from_camera must contain nine values")
    rotation = np.asarray(values, dtype=np.float64).reshape(3, 3)
    orthogonality = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
    determinant = float(np.linalg.det(rotation))
    print(f"orthogonality_error: {orthogonality:.9g}")
    print(f"determinant: {determinant:.9g}")
    print("camera_axes_in_body:")
    for name, vector in zip(("right", "down", "forward"), rotation.T):
        print(f"  {name}: [{vector[0]:.8f}, {vector[1]:.8f}, {vector[2]:.8f}]")
    if orthogonality > args.max_orthogonality_error or abs(determinant - 1.0) > 1.0e-3:
        print("REJECT: matrix is not a proper rotation")
        return 2
    print("PASS: matrix is a proper rotation; still verify axis signs with a fixed target")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
