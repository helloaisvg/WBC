#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys
import tomllib
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
PIPELINE_DIR = PROJECT_ROOT / "research" / "pipeline"


def command_from_study(study, output_path, log_directory):
    dynamic = study.get("dynamic", {})
    map_config = study.get("map", {})
    execution = study.get("execution", {})
    command = [
        sys.executable,
        str(PIPELINE_DIR / "run_matrix.py"),
        "--modes",
        *map(str, study["modes"]),
        "--scenarios",
        *map(str, study["scenarios"]),
        "--seeds",
        *map(str, study["seeds"]),
        "--obs-num",
        str(map_config.get("ground_obstacles", 20)),
        "--float-obs-num",
        str(map_config.get("floating_obstacles", 30)),
        "--dynamic-obs-num",
        str(dynamic.get("count", 0)) if dynamic.get("enabled", False) else "0",
        "--dynamic-speed",
        str(dynamic.get("speed", 0.9)),
        "--dynamic-radius",
        str(dynamic.get("radius", 0.45)),
        "--dynamic-cross-x",
        str(dynamic.get("cross_x", 0.4)),
        "--dynamic-cross-y",
        str(dynamic.get("cross_y", 0.0)),
        "--dynamic-z",
        str(dynamic.get("z", 0.62)),
        "--dynamic-cross-offset",
        str(dynamic.get("cross_offset", 10.0)),
        "--dynamic-spawn-delay",
        str(dynamic.get("spawn_delay", 0.0)),
        "--startup-wait",
        str(execution.get("startup_wait", 5)),
        "--run-seconds",
        str(execution.get("run_seconds", 60)),
        "--output",
        str(output_path),
        "--save-logs-dir",
        str(log_directory),
    ]
    if execution.get("stop_after_second_plan", False):
        command.append("--stop-after-second-plan")
    return command


def run_stage(command, timeout):
    return subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=True,
    ).stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("study", type=Path)
    parser.add_argument("--run-id")
    args = parser.parse_args()

    study_path = args.study.resolve()
    with study_path.open("rb") as stream:
        study = tomllib.load(stream)
    run_id = args.run_id or datetime.now().strftime("%Y%m%dT%H%M%S")
    run_directory = (
        PROJECT_ROOT / "research" / "runs" / study["name"] / run_id
    )
    raw_directory = run_directory / "raw"
    processed_directory = run_directory / "processed"
    figure_directory = run_directory / "report" / "figures"
    raw_directory.mkdir(parents=True, exist_ok=True)
    processed_directory.mkdir(parents=True, exist_ok=True)
    figure_directory.mkdir(parents=True, exist_ok=True)

    manifest = {
        "study": study,
        "study_file": str(study_path.relative_to(PROJECT_ROOT)),
        "run_id": run_id,
        "executed_at": datetime.now().isoformat(timespec="seconds"),
        "project_root": str(PROJECT_ROOT),
    }
    (run_directory / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    )

    timeout = int(study.get("execution", {}).get("timeout_seconds", 3600))
    trials_path = raw_directory / "trials.csv"
    matrix_output = run_stage(
        command_from_study(
            study,
            trials_path,
            raw_directory / "logs",
        ),
        timeout,
    )
    (raw_directory / "run_matrix.log").write_text(matrix_output)

    summary_output = run_stage(
        [
            sys.executable,
            str(PIPELINE_DIR / "summarize.py"),
            str(trials_path),
        ],
        timeout,
    )
    (processed_directory / "summary.md").write_text(summary_output)

    paired_output = run_stage(
        [
            sys.executable,
            str(PIPELINE_DIR / "paired_stats.py"),
            str(trials_path),
            "--full",
            "full",
        ],
        timeout,
    )
    (processed_directory / "paired_stats.md").write_text(paired_output)

    run_stage(
        [
            sys.executable,
            str(PIPELINE_DIR / "plot.py"),
            str(trials_path),
            "--output",
            str(figure_directory / "benchmark.png"),
        ],
        timeout,
    )
    print(run_directory)


if __name__ == "__main__":
    main()
