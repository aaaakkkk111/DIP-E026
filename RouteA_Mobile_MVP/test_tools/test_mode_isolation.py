import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = Path(r"D:\DIP\STM32平衡车_V2\10.附件\源码汇总\0.Large program\1.standard libraries(keil)\stm32_Balance_Car_L")


def text(path: Path) -> str:
    return path.read_bytes().decode("latin1")


def mode_names(path: Path) -> list[str]:
    body = re.search(r"typedef enum Car_mode_t\{(.*?)Mode_Max", text(path), re.S).group(1)
    return re.findall(r"^\s*([A-Za-z][A-Za-z0-9_]*)\s*,", body, re.M)


class TestModeIsolation(unittest.TestCase):
    def test_official_mode_numbers_are_unchanged(self):
        official = mode_names(OFFICIAL / "USER/myenum.h")
        modified = mode_names(ROOT / "firmware/USER/myenum.h")
        self.assertEqual(20, len(official))
        self.assertEqual(official, modified[:20])
        self.assertEqual("Test_Mode", modified[20])

    def test_main_does_not_force_standard_mode(self):
        main = text(ROOT / "firmware/USER/main.c")
        self.assertNotIn("mode = Normal;", main)
        self.assertIn("if(mode == Test_Mode)\r\n\t{\r\n\t\tRouteA_Init();", main)

    def test_control_hooks_are_test_only(self):
        control = text(ROOT / "firmware/APP/app_control.c")
        for hook in ("RouteA_ControlBegin", "RouteA_ControlCapture", "RouteA_ControlEnd"):
            line = next(line for line in control.splitlines() if hook + "(" in line)
            self.assertIn("if(mode == Test_Mode)", line)

    def test_standard_mode_pid_is_shared_only_as_test_baseline(self):
        mode = text(ROOT / "firmware/APP/mode/app_mode.c")
        self.assertIn("mode == Normal || mode == PS2_Control || mode == LiDar_Patrol || mode == Test_Mode", mode)


if __name__ == "__main__":
    unittest.main()
