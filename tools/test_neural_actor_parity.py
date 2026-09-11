"""Compare the generated C actor against Stable-Baselines3 inference."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("executable", type=Path)
    parser.add_argument("--vectors", type=int, default=1000)
    args = parser.parse_args()

    model = PPO.load(args.model, device="cpu")
    rng = np.random.default_rng(20260909)
    observations = rng.uniform(-5.0, 5.0, (args.vectors, 13)).astype(np.float32)
    payload = "".join(" ".join(f"{float(v):.9g}" for v in row) + "\n" for row in observations)
    process = subprocess.run(
        [str(args.executable)],
        input=payload,
        text=True,
        capture_output=True,
        check=True,
    )
    c_outputs = np.loadtxt(process.stdout.splitlines(), dtype=np.float32).reshape(-1, 2)
    python_outputs = np.vstack(
        [model.predict(row, deterministic=True)[0] for row in observations]
    ).astype(np.float32)
    error = np.abs(c_outputs - python_outputs)
    print(f"vectors={len(observations)}")
    print(f"max_abs_error={float(error.max()):.9g}")
    print(f"mean_abs_error={float(error.mean()):.9g}")
    print(f"vectors_within_1e-5={int(np.all(error <= 1e-5, axis=1).sum())}")
    if not np.all(error <= 1e-5):
        raise SystemExit("C actor parity threshold exceeded")


if __name__ == "__main__":
    main()
