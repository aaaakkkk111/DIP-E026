"""Print exactly which environment this process is running in.

Run it the same way you ran the thing that broke -- Run, Run in Dedicated
Terminal, right-click, a run configuration -- and compare the output.  Nearly
every "works here, not there" on this project is one of the four lines under
INTERPRETER or the two under PATHS.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WANT = os.path.join(PROJ, ".venv", "Scripts", "python.exe")


def main():
    print("=" * 62)
    print("INTERPRETER")
    print(f"  可执行文件   {sys.executable}")
    print(f"  版本         {sys.version.split()[0]}")
    in_venv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    print(f"  虚拟环境     {'是' if in_venv else '否 —— 用的是系统/其他 Python'}")
    same = os.path.normcase(os.path.realpath(sys.executable)) == \
        os.path.normcase(os.path.realpath(WANT)) if os.path.exists(WANT) else False
    print(f"  是本工程的   {'是' if same else '★否★  应该是 ' + WANT}")

    print("\nPACKAGES")
    import importlib.util as u
    need = [("numpy", "必需"), ("mujoco", "3D 画面"), ("PyQt6", "界面"),
            ("scipy", "自检里的 LQR 交叉验证"),
            ("gymnasium", "训练"), ("stable_baselines3", "训练"),
            ("torch", "训练"), ("tensorboard", "训练日志")]
    for n, why in need:
        ok = u.find_spec(n) is not None
        print(f"  [{'OK ' if ok else '缺 '}] {n:20s} {why}")

    print("\nPATHS")
    print(f"  工作目录     {os.getcwd()}")
    print(f"  工程目录     {PROJ}")
    print(f"  能 import    ", end="")
    try:
        import balance_bot                                   # noqa: F401
        print(f"balance_bot OK  ({os.path.dirname(balance_bot.__file__)})")
    except Exception as e:
        print(f"★balance_bot 失败★  {e!r}")

    print("\nCONSOLE")
    print(f"  stdout 编码  {sys.stdout.encoding}")
    print(f"  isatty       {sys.stdout.isatty()}  "
          f"(真终端为 True，Run 窗口/管道为 False)")
    print(f"  中文测试     存活步数 俯仰 摔倒率 —— 这行乱码就是编码问题")
    print(f"  PYTHONIOENCODING={os.environ.get('PYTHONIOENCODING', '(未设)')}"
          f"  PYTHONUNBUFFERED={os.environ.get('PYTHONUNBUFFERED', '(未设)')}")
    print("=" * 62)


if __name__ == "__main__":
    main()
