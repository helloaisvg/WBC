#!/usr/bin/env python3

import csv
import statistics
import sys
from collections import defaultdict


def numeric_values(row, fields):
    return [float(row[field]) for field in fields if row[field]]


def main():
    paths = sys.argv[1:] or [
        "research/results/tables/recovery_large_static.csv"
    ]
    groups = defaultdict(list)
    for path in paths:
        with open(path, newline="") as source:
            for row in csv.DictReader(source):
                groups[
                    (
                        row["mode"],
                        row["scenario"],
                        row.get("dynamic_obs_num", "0"),
                        row.get("dynamic_obs_speed", "0"),
                        row.get("obs_num", "default"),
                        row.get("float_obs_num", "default"),
                    )
                ].append(row)

    columns = [
        "Mode", "Scenario", "Dynamic", "Speed", "Generated obstacles",
        "Runs", "Map ready", "Plan runs", "Run success",
        "Collision notices", "Emergency stops", "Replans", "Mean wall ms",
        "P95 wall ms", "Worst margin m", "Worst slack s",
        "Metric deficits", "Repairs",
    ]
    print("| " + " | ".join(columns) + " |")
    print("|" + "|".join(["---"] * len(columns)) + "|")
    for (
        mode,
        scenario,
        dynamic,
        dynamic_speed,
        obs_num,
        float_obs_num,
    ), rows in sorted(groups.items()):
        walls, margins, slacks = [], [], []
        mission_success = collision_notices = emergency_stops = 0
        replans = repairs = plan_runs = map_ready = 0
        violations = trajectories = 0
        generated_ground = []
        generated_floating = []
        for row in rows:
            plans = int(row["plans"])
            plan_runs += plans >= 1
            map_ready += int(row.get("map_loaded", "1"))
            if row.get("stop_after") == "second_plan":
                reached_second_goal = plans >= 2
            elif row.get("second_goal_completed", ""):
                reached_second_goal = int(row["second_goal_completed"]) >= 1
            else:
                reached_second_goal = plans >= 2
            mission_success += (
                reached_second_goal and int(row["emergency_stops"]) == 0
            )
            collision_notices += int(row["collision_warnings"])
            emergency_stops += int(row["emergency_stops"])
            replans += int(row.get("replanning_events", "0"))
            repairs += int(row["temporal_repairs"])
            generated_ground.extend(
                numeric_values(
                    row,
                    ["generated_ground"],
                )
            )
            generated_floating.extend(
                numeric_values(
                    row,
                    ["generated_floating"],
                )
            )
            row_walls = numeric_values(
                row, ["wall_ms_1", "wall_ms_2"]
            )
            row_margins = numeric_values(
                row, ["margin_m_1", "margin_m_2"]
            )
            row_slacks = numeric_values(
                row, ["recovery_slack_s_1", "recovery_slack_s_2"]
            )
            walls.extend(row_walls)
            margins.extend(row_margins)
            slacks.extend(row_slacks)
            trajectories += len(row_walls)
            violations += sum(
                margin < -0.07
                for margin in row_margins
            )
            violations += sum(
                slack < -0.10
                for slack in row_slacks
            )
        walls.sort()
        success = 100.0 * mission_success / len(rows)
        planning_rate = 100.0 * plan_runs / len(rows)
        mean_wall = statistics.mean(walls) if walls else 0.0
        p95_wall = (
            walls[min(len(walls) - 1, int(0.95 * (len(walls) - 1)))]
            if walls else 0.0
        )
        worst_margin = min(margins) if margins else 0.0
        worst_slack = min(slacks) if slacks else 0.0
        ground_mean = (
            statistics.mean(generated_ground) if generated_ground else 0.0
        )
        floating_mean = (
            statistics.mean(generated_floating)
            if generated_floating else 0.0
        )
        print(
            f"| {mode} | {scenario} | {dynamic} | {dynamic_speed} | "
            f"{ground_mean:.1f}/{floating_mean:.1f} | "
            f"{len(rows)} | {map_ready}/{len(rows)} | "
            f"{planning_rate:.1f}% | {success:.1f}% | "
            f"{collision_notices} | {emergency_stops} | {replans} | "
            f"{mean_wall:.1f} | {p95_wall:.1f} | {worst_margin:.4f} | "
            f"{worst_slack:.4f} | {violations} | {repairs} |"
        )


if __name__ == "__main__":
    main()
