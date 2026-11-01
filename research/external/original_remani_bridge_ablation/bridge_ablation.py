#!/usr/bin/env python3
import argparse
import csv
import math
import os
import statistics
import subprocess
from pathlib import Path


HEIGHTS = (0.7, 0.6, 0.5, 0.4)
MODES = ("none", "global_single", "local_single", "local_multi")


def percentile(values, fraction):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def parse_results(lines):
    rows = []
    for line in lines:
        if not line.startswith("BRIDGE_RESULT,"):
            continue
        values = {}
        for field in line.strip().split(",")[1:]:
            key, value = field.split("=", 1)
            values[key] = value
        rows.append(
            {
                "height_m": float(values["height"]),
                "mode": values["mode"],
                "frontend_only": int(values["frontend_only"]),
                "trial": int(values["trial"]),
                "seed": int(values["seed"]),
                "start_y": float(values["start_y"]),
                "goal_y": float(values["goal_y"]),
                "success": int(values["success"]),
                "planned": int(values["planned"]),
                "within_budget": int(values["within_budget"]),
                "wall_ms": float(values["wall_ms"]),
                "init_ms": float(values["init_ms"]),
                "optimize_ms": float(values["optimize_ms"]),
            }
        )
    return rows


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row["height_m"], row["mode"]), []).append(row)

    summary = []
    for height in HEIGHTS:
        for mode in MODES:
            group = groups[(height, mode)]
            successful = [row["wall_ms"] for row in group if row["success"]]
            all_times = [row["wall_ms"] for row in group]
            planned = [row for row in group if row["planned"]]
            summary.append(
                {
                    "height_m": height,
                    "mode": mode,
                    "trials": len(group),
                    "successes": sum(row["success"] for row in group),
                    "success_rate_pct": 100.0
                    * sum(row["success"] for row in group)
                    / len(group),
                    "budget_failures": sum(
                        1 for row in group if row["planned"] and not row["success"]
                    ),
                    "mean_wall_ms": statistics.fmean(all_times),
                    "p95_wall_ms": percentile(all_times, 0.95),
                    "mean_success_ms": statistics.fmean(successful)
                    if successful
                    else math.nan,
                    "p95_success_ms": percentile(successful, 0.95),
                    "mean_init_ms": statistics.fmean(
                        row["init_ms"] for row in planned
                    )
                    if planned
                    else math.nan,
                    "mean_optimize_ms": statistics.fmean(
                        row["optimize_ms"] for row in planned
                    )
                    if planned
                    else math.nan,
                }
            )
    return summary


def write_csv(path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--container", default="lns-ablation")
    parser.add_argument("--trials", type=int, default=25)
    parser.add_argument("--batch-size", type=int, default=5)
    parser.add_argument("--timeout", type=int, default=150)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "results" / "bridge_ablation",
    )
    args = parser.parse_args()

    raw_rows = []
    log_dir = args.output / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    for height in HEIGHTS:
        for mode in MODES:
            rows = []
            batch_count = math.ceil(args.trials / args.batch_size)
            for batch in range(batch_count):
                batch_start = batch * args.batch_size
                batch_trials = min(args.batch_size, args.trials - batch_start)
                launch = (
                    "source /opt/ros/noetic/setup.bash && "
                    "source /ws/devel/setup.bash && "
                    "export CPATH=/opt/ros/noetic/include/ompl-1.6:${CPATH} && "
                    f"timeout --signal=INT --kill-after=5s {args.timeout}s "
                    "roslaunch remani_planner bridge_ablation.launch "
                    f"bridge_height:={height:.1f} fallback_mode:={mode} "
                    f"trials:={batch_trials} seed_offset:={batch_start} "
                    "budget_ms:=600"
                )
                completed = subprocess.run(
                    ["docker", "exec", args.container, "bash", "-lc", launch],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=args.timeout + 10,
                    check=False,
                )
                log_path = log_dir / (
                    f"bridge_{height:.1f}_{mode}_batch_{batch:02d}.log"
                )
                log_path.write_text(completed.stdout, encoding="utf-8")
                batch_rows = parse_results(completed.stdout.splitlines())
                if len(batch_rows) != batch_trials:
                    raise RuntimeError(
                        f"{height:.1f} m {mode} batch {batch}: expected "
                        f"{batch_trials} results, found {len(batch_rows)}; "
                        f"log={log_path}"
                    )
                rows.extend(batch_rows)
            if len(rows) != args.trials:
                raise RuntimeError(
                    f"{height:.1f} m {mode}: expected {args.trials} results, "
                    f"found {len(rows)}"
                )
            raw_rows.extend(rows)
            successes = sum(row["success"] for row in rows)
            print(
                f"height={height:.1f} mode={mode} "
                f"success={successes}/{args.trials}",
                flush=True,
            )

    summary = summarize(raw_rows)
    write_csv(
        args.output / "raw.csv",
        raw_rows,
        [
            "height_m",
            "mode",
            "frontend_only",
            "trial",
            "seed",
            "start_y",
            "goal_y",
            "success",
            "planned",
            "within_budget",
            "wall_ms",
            "init_ms",
            "optimize_ms",
        ],
    )
    write_csv(
        args.output / "summary.csv",
        summary,
        [
            "height_m",
            "mode",
            "trials",
            "successes",
            "success_rate_pct",
            "budget_failures",
            "mean_wall_ms",
            "p95_wall_ms",
            "mean_success_ms",
            "p95_success_ms",
            "mean_init_ms",
            "mean_optimize_ms",
        ],
    )


if __name__ == "__main__":
    main()
