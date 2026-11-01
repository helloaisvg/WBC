#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import binomtest, wilcoxon


KEY_COLUMNS = [
    "scenario",
    "seed",
    "dynamic_obs_num",
    "dynamic_obs_speed",
    "dynamic_obs_cross_x",
    "dynamic_obs_cross_y",
    "dynamic_obs_spawn_delay",
    "obs_num",
    "float_obs_num",
]


def load_rows(paths):
    frames = [pd.read_csv(path) for path in paths]
    data = pd.concat(frames, ignore_index=True)
    for column in KEY_COLUMNS:
        if column not in data:
            data[column] = 0
    data = data.drop_duplicates(
        subset=KEY_COLUMNS + ["mode"],
        keep="last",
    )
    for column in (
        "wall_ms_1", "wall_ms_2",
        "margin_m_1", "margin_m_2",
        "recovery_slack_s_1", "recovery_slack_s_2",
        "plans", "emergency_stops", "second_goal_completed",
    ):
        if column in data:
            data[column] = pd.to_numeric(data[column], errors="coerce")
    return data


def mission_success(row):
    if row.get("stop_after") == "second_plan":
        reached = int(row["plans"]) >= 2
    elif "second_goal_completed" in row and pd.notna(row["second_goal_completed"]):
        reached = int(row["second_goal_completed"]) >= 1
    else:
        reached = int(row["plans"]) >= 2
    return int(reached and int(row["emergency_stops"]) == 0)


def trajectory_metrics(data):
    records = []
    for _, row in data.iterrows():
        key = tuple(row[column] for column in KEY_COLUMNS)
        for plan_index, suffix in enumerate(("1", "2"), start=1):
            wall = row.get(f"wall_ms_{suffix}", np.nan)
            if pd.isna(wall):
                continue
            records.append(
                {
                    **dict(zip(KEY_COLUMNS, key)),
                    "mode": row["mode"],
                    "plan_index": plan_index,
                    "wall_ms": wall,
                    "margin": row.get(f"margin_m_{suffix}", np.nan),
                    "slack": row.get(f"recovery_slack_s_{suffix}", np.nan),
                }
            )
    return pd.DataFrame(records)


def compare_metric(metrics, baseline, full, metric):
    left = metrics[metrics["mode"] == baseline][
        KEY_COLUMNS + ["plan_index", metric]
    ].dropna()
    right = metrics[metrics["mode"] == full][
        KEY_COLUMNS + ["plan_index", metric]
    ].dropna()
    paired = left.merge(
        right,
        on=KEY_COLUMNS + ["plan_index"],
        suffixes=("_baseline", "_full"),
    )
    if len(paired) < 6:
        return len(paired), float("nan"), float("nan")
    delta = paired[f"{metric}_full"] - paired[f"{metric}_baseline"]
    if np.allclose(delta, 0.0):
        p_value = 1.0
    else:
        p_value = float(wilcoxon(delta, alternative="two-sided").pvalue)
    return len(paired), float(np.median(delta)), p_value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--full", default="full")
    args = parser.parse_args()

    data = load_rows(args.inputs)
    data["mission_success"] = data.apply(mission_success, axis=1)
    metrics = trajectory_metrics(data)
    baselines = [
        mode for mode in sorted(data["mode"].dropna().unique())
        if mode != args.full
    ]

    print(
        "| Baseline | Metric | Paired samples | Median delta (full - baseline) | "
        "Wilcoxon p |"
    )
    print("|---|---|---:|---:|---:|")
    for baseline in baselines:
        for metric, label in (
            ("wall_ms", "Wall time [ms]"),
            ("margin", "Braking margin [m]"),
            ("slack", "Recovery slack [s]"),
        ):
            count, median_delta, p_value = compare_metric(
                metrics, baseline, args.full, metric
            )
            print(
                f"| {baseline} | {label} | {count} | "
                f"{median_delta:.4f} | {p_value:.4g} |"
            )

    print()
    print("| Baseline | Scenario | Full-only success | Baseline-only success | p |")
    print("|---|---|---:|---:|---:|")
    for baseline in baselines:
        left = data[data["mode"] == baseline][
            KEY_COLUMNS + ["mission_success"]
        ]
        right = data[data["mode"] == args.full][
            KEY_COLUMNS + ["mission_success"]
        ]
        paired = left.merge(
            right,
            on=KEY_COLUMNS,
            suffixes=("_baseline", "_full"),
        )
        for scenario in sorted(paired["scenario"].unique()):
            subset = paired[paired["scenario"] == scenario]
            full_only = int(
                ((subset["mission_success_full"] == 1) &
                 (subset["mission_success_baseline"] == 0)).sum()
            )
            baseline_only = int(
                ((subset["mission_success_full"] == 0) &
                 (subset["mission_success_baseline"] == 1)).sum()
            )
            discordant = full_only + baseline_only
            p_value = (
                float(binomtest(min(full_only, baseline_only),
                                discordant, 0.5).pvalue)
                if discordant else 1.0
            )
            print(
                f"| {baseline} | {scenario} | {full_only} | "
                f"{baseline_only} | {p_value:.4g} |"
            )


if __name__ == "__main__":
    main()
