#!/usr/bin/env python3

import runpy
from pathlib import Path


TARGET = (
    Path(__file__).resolve().parents[1]
    / "research"
    / "scenarios"
    / "compare_dynamic_scenario.py"
)
runpy.run_path(str(TARGET), run_name="__main__")
