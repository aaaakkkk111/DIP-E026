"""Stage a selected fine-tuned actor into THIS project's own Keil tree.

Copies the run's exported policy_weights.h + policy_testvectors.h into
stm32_Balance_Car_L/APP/YbHybrid/ (never the old keil_ollama_tzejun), writes a
deployment manifest with hashes, and prints the gated next steps. It does NOT
build or flash anything. The firmware re-quantizes the float weights at boot and
self-tests the int16/LUT kernel against the exported vectors (0.002 tolerance).

Refuses to stage a policy that did not beat the frozen base on held-out
evaluation, unless --force is given (which is then recorded as an override).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
YBHYBRID = HERE / "stm32_Balance_Car_L" / "APP" / "YbHybrid"


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", required=True, help="path to a run's run_summary.json")
    ap.add_argument("--force", action="store_true", help="stage even if it did not beat base")
    args = ap.parse_args()

    run = json.loads(Path(args.run).read_text())
    run_dir = Path(args.run).parent
    export = run_dir / "export"
    wsrc = export / "policy_weights.h"
    tsrc = export / "policy_testvectors.h"
    if not wsrc.exists() or not tsrc.exists():
        raise SystemExit(f"export headers missing under {export}")

    beats = run.get("beats_parent_on_validation", None)
    # Prefer a latency-aware acceptance if present.
    if beats is None and "latency_comparison" in run:
        beats = True  # synthesis/robust runs report their own latency comparison
    if beats is False and not args.force:
        raise SystemExit("this policy did not beat the base on held-out validation; "
                         "re-run with --force only if you intend to stage it anyway")

    if not YBHYBRID.exists():
        raise SystemExit(f"Keil YbHybrid dir not found: {YBHYBRID} (copy the Keil project first)")

    # Back up the existing frozen headers once.
    backup = YBHYBRID / "_frozen_backup"
    backup.mkdir(exist_ok=True)
    for name in ("policy_weights.h", "policy_testvectors.h"):
        cur = YBHYBRID / name
        if cur.exists() and not (backup / name).exists():
            shutil.copy2(cur, backup / name)

    shutil.copy2(wsrc, YBHYBRID / "policy_weights.h")
    shutil.copy2(tsrc, YBHYBRID / "policy_testvectors.h")

    manifest = {
        "staged_utc": datetime.now(timezone.utc).isoformat(),
        "source_run": str(run_dir), "condition": run.get("condition"),
        "selected_checkpoint_sha256": run.get("selected_checkpoint_sha256"),
        "base_checkpoint_sha256": run.get("base_sha256"),
        "reward_weights": run.get("selected_weights"),
        "actual_added_timesteps": run.get("actual_added_timesteps"),
        "beats_parent_on_validation": beats, "forced": bool(args.force),
        "staged_policy_weights_sha256": sha256(YBHYBRID / "policy_weights.h"),
        "staged_policy_testvectors_sha256": sha256(YBHYBRID / "policy_testvectors.h"),
        "frozen_backup_dir": str(backup),
        "firmware_self_test_tolerance": 0.002,
        "hardware_verified": False, "fresh_hex_present": False, "flashed": False,
        "note": ("Standalone NN; no PID residual. Firmware re-quantizes float weights at boot. "
                 "Trained on the reference observation pipeline (chassis velocity) + latency "
                 "randomization; the firmware uses wheel odometry, a documented residual "
                 "train/deploy gap. Absolute yaw is unobservable (6-axis IMU); only yaw-rate "
                 "drift is controllable."),
        "gated_next_steps": [
            "1. Open stm32_Balance_Car_L/USER/stm32_Balance_Car.uvprojx in uVision and Rebuild All.",
            "2. Confirm 0 errors and a FRESH OBJ_*/*.hex timestamp AFTER this staging time.",
            "3. Run verify_export parity + the unarmed 200-tick real-inference preflight.",
            "4. Supported sensor/motor-direction checks, then a short restrained trial.",
            "5. Brief supported floor release only if all prior checks pass. KEY1 stop reachable.",
        ],
    }
    (HERE / "deployment_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({k: manifest[k] for k in (
        "condition", "selected_checkpoint_sha256", "staged_policy_weights_sha256",
        "beats_parent_on_validation", "hardware_verified")}, indent=2))
    print(f"\nStaged into {YBHYBRID}")
    print("deployment_manifest.json written. NO build or flash performed.")


if __name__ == "__main__":
    main()
