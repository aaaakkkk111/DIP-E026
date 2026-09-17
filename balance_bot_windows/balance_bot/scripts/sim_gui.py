import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.ui.sim_gui import run_ui

if __name__ == "__main__":
    run_ui()
