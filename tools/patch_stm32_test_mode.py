"""Apply the isolated TEST-mode integration to the staged vendor project."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(r"D:\DIP\Simulation\stm32_test_migration")


def read_legacy(path: Path) -> tuple[str, str]:
    data = path.read_bytes()
    for encoding in ("utf-8", "gb18030", "latin-1"):
        try:
            return data.decode(encoding), encoding
        except UnicodeDecodeError:
            pass
    raise RuntimeError(f"cannot decode {path}")


def write_legacy(path: Path, text: str, encoding: str) -> None:
    path.write_bytes(text.encode(encoding))


def replace_once(text: str, old: str, new: str, path: Path) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected one occurrence in {path}: {old!r}; found {count}")
    return text.replace(old, new, 1)


def insert_after_line(text: str, needle: str, inserted: str, path: Path) -> str:
    lines = text.splitlines(keepends=True)
    indexes = [index for index, line in enumerate(lines) if needle in line]
    if len(indexes) != 1:
        raise RuntimeError(f"expected one line in {path}: {needle!r}; found {len(indexes)}")
    newline = "\r\n" if lines[indexes[0]].endswith("\r\n") else "\n"
    lines.insert(indexes[0] + 1, inserted + newline)
    return "".join(lines)


def patch_text(relative: str, transform) -> None:
    path = ROOT / relative
    text, encoding = read_legacy(path)
    updated = transform(text, path)
    if updated == text:
        raise RuntimeError(f"patch made no change: {path}")
    write_legacy(path, updated, encoding)


def patch_enum(text: str, path: Path) -> str:
    return insert_after_line(
        text,
        "ElE_Mode,",
        "\tTEST_Mode,//Neural residual assisted control test mode",
        path,
    )


def patch_headers(text: str, path: Path) -> str:
    return replace_once(
        text,
        '#include "pid_control.h"',
        '#include "pid_control.h"\r\n\r\n//TEST neural residual assisted control\r\n'
        '#include "test_assist.h"\r\n#include "neural_actor.h"',
        path,
    )


def patch_oled(text: str, path: Path) -> str:
    return insert_after_line(
        text,
        "case ElE_Mode:",
        '\t\tcase TEST_Mode: OLED_Draw_Line("21.TEST Neural", 1, true, true); break;',
        path,
    )


def patch_mode(text: str, path: Path) -> str:
    text = insert_after_line(
        text,
        "case Diff_Line_track:",
        "\t\tcase TEST_Mode:",
        path,
    )
    text = replace_once(
        text,
        "if(mode == Normal || mode == Weight_M)",
        "if(mode == Normal || mode == Weight_M || mode == TEST_Mode)",
        path,
    )
    text = replace_once(
        text,
        "else if(mode == Normal || mode == PS2_Control || mode == LiDar_Patrol)",
        "else if(mode == Normal || mode == PS2_Control || mode == LiDar_Patrol || mode == TEST_Mode)",
        path,
    )
    return text


def patch_bsp(text: str, path: Path) -> str:
    text = replace_once(
        text,
        "if(mode == Normal || mode == Weight_M)",
        "if(mode == Normal || mode == Weight_M || mode == TEST_Mode)",
        path,
    )
    lines = text.splitlines(keepends=True)
    indexes = [index for index, line in enumerate(lines) if "TIM2_Cap_Init(0XFFFF,72-1);" in line]
    if len(indexes) < 1:
        raise RuntimeError(f"TIM2 initialization not found in {path}")
    index = indexes[0]
    newline = "\r\n" if lines[index].endswith("\r\n") else "\n"
    addition = (
        "\t\tif(mode == TEST_Mode)" + newline
        + "\t\t{" + newline
        + "\t\t\tTestAssist_Init();" + newline
        + "\t\t}" + newline
    )
    lines.insert(index + 1, addition)
    return "".join(lines)


def patch_timer(text: str, path: Path) -> str:
    text = replace_once(
        text,
        "mode == U_Avoid || mode == Weight_M)",
        "mode == U_Avoid || mode == Weight_M || mode == TEST_Mode)",
        path,
    )
    return replace_once(
        text,
        "if(mode == Normal || mode == Weight_M)",
        "if(mode == Normal || mode == Weight_M || mode == TEST_Mode)",
        path,
    )


def patch_main(text: str, path: Path) -> str:
    return replace_once(
        text,
        "if(mode == Normal || mode == Weight_M)",
        "if(mode == Normal || mode == Weight_M || mode == TEST_Mode)",
        path,
    )


def patch_control(text: str, path: Path) -> str:
    lines = text.splitlines(keepends=True)
    start_matches = [i for i, line in enumerate(lines) if "Balance_Pwm=Balance_PD" in line]
    end_matches = [i for i, line in enumerate(lines) if "Motor_Right=PWM_Limit(Motor_Right,2600,-2600);" in line]
    if len(start_matches) != 1 or len(end_matches) != 1:
        raise RuntimeError(f"control block markers not unique in {path}")
    start = start_matches[0]
    end = end_matches[0]
    if end <= start:
        raise RuntimeError(f"invalid control block in {path}")
    newline = "\r\n" if lines[start].endswith("\r\n") else "\n"
    block = [
        "\t\tif(mode == TEST_Mode)",
        "\t\t{",
        "\t\t\tTestAssist_ControlStep(Angle_Balance,",
        "\t\t\t                       Gyro_Balance,",
        "\t\t\t                       Gyro_Turn,",
        "\t\t\t                       Encoder_Left,",
        "\t\t\t                       Encoder_Right,",
        "\t\t\t                       &Motor_Left,",
        "\t\t\t                       &Motor_Right);",
        "\t\t}",
        "\t\telse",
        "\t\t{",
        "\t\t\tBalance_Pwm=Balance_PD(Angle_Balance,Gyro_Balance);",
        "\t\t\tVelocity_Pwm=Velocity_PI(Encoder_Left,Encoder_Right);",
        "",
        "\t\t\tif(mode == Line_Track || mode == Diff_Line_track)",
        "\t\t\t{",
        "\t\t\t\tTurn_Pwm=Turn_IRTrack_PD(Gyro_Turn);",
        "\t\t\t}",
        "\t\t\telse if(mode == CCD_Mode)",
        "\t\t\t{",
        "\t\t\t\tTurn_Pwm=Turn_CCD_PD(Gyro_Turn);",
        "\t\t\t}",
        "\t\t\telse if(mode == ElE_Mode)",
        "\t\t\t{",
        "\t\t\t\tTurn_Pwm=Turn_ELE_PD(Gyro_Turn);",
        "\t\t\t}",
        "\t\t\telse if(mode == K210_Line)",
        "\t\t\t{",
        "\t\t\t\tTurn_Pwm=Turn_K210_PD(Gyro_Turn);",
        "\t\t\t}",
        "\t\t\telse",
        "\t\t\t{",
        "\t\t\t\tTurn_Pwm=Turn_PD(Gyro_Turn);",
        "\t\t\t}",
        "",
        "\t\t\tMotor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;",
        "\t\t\tMotor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;",
        "\t\t\tMotor_Left=PWM_Ignore(Motor_Left);",
        "\t\t\tMotor_Right=PWM_Ignore(Motor_Right);",
        "\t\t\tMotor_Left=PWM_Limit(Motor_Left,2600,-2600);",
        "\t\t\tMotor_Right=PWM_Limit(Motor_Right,2600,-2600);",
        "\t\t}",
    ]
    lines[start : end + 1] = [(line + newline) for line in block]
    return "".join(lines)


def patch_project(text: str, path: Path) -> str:
    text = replace_once(
        text,
        "..\\APP\\Lidar</IncludePath>",
        "..\\APP\\Lidar;..\\APP\\TestAssist</IncludePath>",
        path,
    )
    marker = "        <Group>\r\n          <GroupName>DMP</GroupName>"
    if marker not in text:
        marker = "        <Group>\n          <GroupName>DMP</GroupName>"
    newline = "\r\n" if "\r\n" in text else "\n"
    group = newline.join(
        [
            "        <Group>",
            "          <GroupName>TEST Assist</GroupName>",
            "          <Files>",
            "            <File>",
            "              <FileName>test_assist.c</FileName>",
            "              <FileType>1</FileType>",
            "              <FilePath>..\\APP\\TestAssist\\test_assist.c</FilePath>",
            "            </File>",
            "            <File>",
            "              <FileName>neural_actor.c</FileName>",
            "              <FileType>1</FileType>",
            "              <FilePath>..\\APP\\TestAssist\\neural_actor.c</FilePath>",
            "            </File>",
            "          </Files>",
            "        </Group>",
        ]
    ) + newline
    return replace_once(text, marker, group + marker, path)


def main() -> None:
    patch_text(r"USER\myenum.h", patch_enum)
    patch_text(r"USER\AllHeader.h", patch_headers)
    patch_text(r"APP\OLED_Show\oled_show.c", patch_oled)
    patch_text(r"APP\mode\app_mode.c", patch_mode)
    patch_text(r"BSP\bsp.c", patch_bsp)
    patch_text(r"BSP\Timer\bsp_timer.c", patch_timer)
    patch_text(r"USER\main.c", patch_main)
    patch_text(r"APP\app_control.c", patch_control)
    patch_text(r"USER\stm32_Balance_Car.uvprojx", patch_project)
    print("TEST mode integration applied to staged project")


if __name__ == "__main__":
    main()
