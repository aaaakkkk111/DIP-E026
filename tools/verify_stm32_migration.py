"""Static acceptance checks for the staged STM32 TEST-mode migration."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path
from xml.etree import ElementTree


WORKSPACE = Path(r"D:\DIP\Simulation")
PROJECT = WORKSPACE / "stm32_test_migration"
MODEL = WORKSPACE / "neural_policy" / "hybrid_residual_ppo.zip"
EXPECTED_MODEL_SHA256 = "9b92948f15cf7ad4da3ef3f69453277a920563365ac5519cf3eb1b231e7c2026"


def legacy_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8", "gb18030", "latin-1"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    raise RuntimeError(f"cannot decode {path}")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main() -> None:
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    require(digest == EXPECTED_MODEL_SHA256, "trained model hash changed")

    enum_text = legacy_text(PROJECT / "USER" / "myenum.h")
    mode_block = enum_text.split("typedef enum Car_mode_t{", 1)[1].split("}Car_Mode;", 1)[0]
    mode_names = []
    for line in mode_block.splitlines():
        code = line.split("//", 1)[0].strip()
        match = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s*,?", code)
        if match:
            mode_names.append(match.group(1))
    require(mode_names[-2:] == ["TEST_Mode", "Mode_Max"], "TEST mode is not appended")
    require(mode_names.index("TEST_Mode") == 20, "existing 20 mode values were not preserved")

    assist = legacy_text(PROJECT / "APP" / "TestAssist" / "test_assist.c")
    require("#define TEST_ASSIST_APPLY_RESIDUAL 0" in assist, "shadow mode is not locked")
    require("TEST_SWITCH_TO_WEIGHT_DEG 10.0f" in assist, "10 degree threshold missing")
    require("TEST_SWITCH_TO_NORMAL_DEG 7.0f" in assist, "7 degree threshold missing")
    require("TEST_RESIDUAL_PWM_LIMIT 350.0f" in assist, "350 PWM bound missing")

    actor = legacy_text(PROJECT / "APP" / "TestAssist" / "neural_actor.c")
    require(EXPECTED_MODEL_SHA256 in actor, "actor provenance hash missing")
    sizes = [int(value) for value in re.findall(r"static const float actor_[wb][0-2]\[(\d+)\]", actor)]
    require(sum(sizes) == 1570, f"unexpected actor parameter count: {sum(sizes)}")

    bsp = legacy_text(PROJECT / "BSP" / "bsp.c")
    require(bsp.count("TestAssist_Init();") == 1, "TEST initialization is not unique")
    control = legacy_text(PROJECT / "APP" / "app_control.c")
    require(control.count("TestAssist_ControlStep(") == 1, "TEST control branch is not unique")
    require(control.index("TestAssist_ControlStep(") < control.index("Turn_Off(Angle_Balance,battery)"),
            "safety check is not after the TEST controller")

    project_xml = ElementTree.parse(PROJECT / "USER" / "stm32_Balance_Car.uvprojx")
    paths = [node.text for node in project_xml.findall(".//FilePath")]
    require(r"..\APP\TestAssist\test_assist.c" in paths, "test_assist.c missing from Keil project")
    require(r"..\APP\TestAssist\neural_actor.c" in paths, "neural_actor.c missing from Keil project")

    print("model_sha256=OK")
    print("mode_values=20 existing + TEST + Mode_Max")
    print("shadow_output_lock=OK")
    print("actor_parameters=1570")
    print("safety_order=OK")
    print("keil_project_entries=OK")


if __name__ == "__main__":
    main()
