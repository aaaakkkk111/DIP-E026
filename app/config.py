from __future__ import annotations
import json
from pathlib import Path
from app.reward.spec import RewardSpec
def load_config(path):
    data=json.loads(Path(path).read_text(encoding="utf8"))
    if data.get("backend") not in ("simulation","replay","hardware","mock_hardware"):raise ValueError("invalid backend")
    if data.get("controller",{}).get("type") not in ("pid","cascaded_pid","lqr","ppo"):raise ValueError("invalid controller")
    RewardSpec(**data.get("reward",{})).validate()
    return data
