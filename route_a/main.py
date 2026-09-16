"""CLI entry point for Route A UI and deterministic offline demo."""

from __future__ import annotations

import argparse
import json

from .deepseek_client import DeepSeekClient, MockLLM
from .firmware_profile import EnvironmentConfig
from .orchestrator import RouteAOrchestrator


def offline_demo(duration: float) -> int:
    orchestrator=RouteAOrchestrator(llm=MockLLM()); env=EnvironmentConfig(imu_noise_deg=0.0,imu_delay_ms=0.0)
    try:
        pending=orchestrator.begin_cycle(env,duration)
        print(json.dumps({"llm":pending.llm.proposal.as_dict(),"harness":{
            "accepted":pending.validation.accepted,"candidate":pending.validation.candidate.as_dict() if pending.validation.candidate else None,
            "reasons":pending.validation.reasons}},indent=2,ensure_ascii=False))
        if pending.validation.accepted:
            result=orchestrator.approve(duration); print(json.dumps(asdict_safe(result),indent=2,ensure_ascii=False))
        return 0
    finally: orchestrator.close()


def asdict_safe(value):
    from dataclasses import asdict
    return asdict(value)


def main() -> int:
    parser=argparse.ArgumentParser(description="Route A LLM-assisted PID tuning")
    parser.add_argument("--offline-demo",action="store_true"); parser.add_argument("--real-api",action="store_true")
    parser.add_argument("--no-viewer",action="store_true",help="do not automatically open the separate MuJoCo 3D window")
    parser.add_argument("--duration",type=float,default=4.0); args=parser.parse_args()
    if args.offline_demo: return offline_demo(args.duration)
    from .ui import launch_ui
    launch_ui(use_real_api=args.real_api,auto_open_viewer=not args.no_viewer); return 0


if __name__=="__main__": raise SystemExit(main())
