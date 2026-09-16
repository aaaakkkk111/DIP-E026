"""Explicit real-API smoke test; never imported by the unit test suite."""

from __future__ import annotations

import argparse
import json

from .deepseek_client import DeepSeekClient
from .firmware_profile import DEFAULT_PID


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--run-real-api",action="store_true"); args=parser.parse_args()
    if not args.run_real_api: parser.error("real network/API use requires --run-real-api")
    summary={"current_pid":DEFAULT_PID.as_dict(),"stage":"balance","environment":{},"trial_command":"stop",
             "data_quality":{"loss_ratio":0},"metrics":{"pitch_rms_deg":2,"pitch_peak_deg":5,"pitch_rate_rms_deg_s":10},
             "score":70,"recent_history":[],"harness_limits":{"max_relative_change":.12}}
    result=DeepSeekClient().propose(summary); print(json.dumps(result.proposal.as_dict(),indent=2)); return 0


if __name__=="__main__": raise SystemExit(main())
