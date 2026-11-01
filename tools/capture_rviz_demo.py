#!/usr/bin/env python3

import runpy
from pathlib import Path


TARGET = (
    Path(__file__).resolve().parents[1]
    / "research"
    / "capture"
    / "capture_rviz_demo.py"
)
runpy.run_path(str(TARGET), run_name="__main__")
