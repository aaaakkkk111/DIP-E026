"""Qt binding shim: PyQt6 -> PySide6 -> PyQt5, whichever is installed."""
from __future__ import annotations

BINDING = None
QtCore = QtGui = QtWidgets = None

for _name in ("PyQt6", "PySide6", "PyQt5"):
    try:
        if _name == "PyQt6":
            from PyQt6 import QtCore, QtGui, QtWidgets  # noqa: F401
        elif _name == "PySide6":
            from PySide6 import QtCore, QtGui, QtWidgets  # noqa: F401
        else:
            from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: F401
        BINDING = _name
        break
    except ImportError:
        continue

if BINDING is None:
    raise ImportError(
        "No Qt binding found.  Install one of:\n"
        "    pip install PyQt6\n"
        "    pip install PySide6\n"
        "    sudo apt install python3-pyqt5")

Qt = QtCore.Qt


# PyQt6 / PySide6 moved every enum into a scoped namespace; these helpers hide
# the difference so the panel code reads the same on all three bindings.
def _enum(root, scoped: str, flat: str):
    obj = root
    for part in scoped.split("."):
        if not hasattr(obj, part):
            return getattr(root, flat)
        obj = getattr(obj, part)
    return obj


ALIGN_CENTER = _enum(Qt, "AlignmentFlag.AlignCenter", "AlignCenter")
ALIGN_LEFT = _enum(Qt, "AlignmentFlag.AlignLeft", "AlignLeft")
ALIGN_RIGHT = _enum(Qt, "AlignmentFlag.AlignRight", "AlignRight")
ORIENT_H = _enum(Qt, "Orientation.Horizontal", "Horizontal")
KEY = Qt.Key if hasattr(Qt, "Key") else Qt
ANTIALIAS = _enum(QtGui.QPainter, "RenderHint.Antialiasing", "Antialiasing")
SOLID = _enum(Qt, "PenStyle.SolidLine", "SolidLine")
DASH = _enum(Qt, "PenStyle.DashLine", "DashLine")
NOPEN = _enum(Qt, "PenStyle.NoPen", "NoPen")
FOCUS_STRONG = _enum(Qt, "FocusPolicy.StrongFocus", "StrongFocus")
FOCUS_NONE = _enum(Qt, "FocusPolicy.NoFocus", "NoFocus")
EV_KEYPRESS = _enum(QtCore.QEvent, "Type.KeyPress", "KeyPress")
EV_KEYRELEASE = _enum(QtCore.QEvent, "Type.KeyRelease", "KeyRelease")
ALIGN_VCENTER = _enum(Qt, "AlignmentFlag.AlignVCenter", "AlignVCenter")
ASPECT_KEEP = _enum(Qt, "AspectRatioMode.KeepAspectRatio",
                    "KeepAspectRatio")
TRANSFORM_SMOOTH = _enum(Qt, "TransformationMode.SmoothTransformation",
                         "SmoothTransformation")
FMT_RGB888 = _enum(QtGui.QImage, "Format.Format_RGB888", "Format_RGB888")
SIZE_EXPANDING = _enum(QtWidgets.QSizePolicy, "Policy.Expanding",
                       "Expanding")
SCROLLBAR_OFF = _enum(Qt, "ScrollBarPolicy.ScrollBarAlwaysOff",
                      "ScrollBarAlwaysOff")


def exec_app(app):
    return app.exec() if hasattr(app, "exec") else app.exec_()
