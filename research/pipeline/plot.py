#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


MODE_ORDER = [
    "remani",
    "current_only",
    "braking",
    "whole_body",
    "static_shell",
    "full",
]


def mission_success(row):
    if row.get("stop_after") == "second_plan":
        reached = int(row["plans"]) >= 2
    elif "second_goal_completed" in row and pd.notna(row["second_goal_completed"]):
        reached = int(row["second_goal_completed"]) >= 1
    else:
        reached = int(row["plans"]) >= 2
    return reached and int(row["emergency_stops"]) == 0


def planning_values(row):
    return [
        float(row[f"wall_ms_{suffix}"])
        for suffix in ("1", "2")
        if pd.notna(row.get(f"wall_ms_{suffix}", np.nan))
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    data = pd.concat(
        [pd.read_csv(path) for path in args.inputs],
        ignore_index=True,
    )
    data["mission_success"] = data.apply(mission_success, axis=1).astype(float)
    data["wall_ms"] = data.apply(
        lambda row: np.mean(planning_values(row))
        if planning_values(row)
        else np.nan,
        axis=1,
    )
    data["margin"] = data[["margin_m_1", "margin_m_2"]].min(axis=1)
    data["slack"] = data[["recovery_slack_s_1", "recovery_slack_s_2"]].min(axis=1)

    scenarios = list(dict.fromkeys(data["scenario"]))
    modes = [mode for mode in MODE_ORDER if mode in set(data["mode"])]
    palette = {
        mode: color for mode, color in zip(
            modes,
            ["#607d8b", "#ff8a65", "#42a5f5", "#26a69a",
             "#8d6e63", "#66bb6a"],
        )
    }

    figure, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    figure.patch.set_facecolor("#f7f9fb")
    for axis in axes.flat:
        axis.set_facecolor("white")
        axis.grid(axis="y", color="#dce4ea", linewidth=0.7)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)

    bar_width = 0.8 / max(1, len(modes))
    positions = np.arange(len(scenarios))
    for mode_index, mode in enumerate(modes):
        subset = data[data["mode"] == mode]
        rates = [
            100.0 * subset[subset["scenario"] == scenario][
                "mission_success"
            ].mean()
            for scenario in scenarios
        ]
        axes[0, 0].bar(
            positions + (mode_index - (len(modes) - 1) / 2) * bar_width,
            rates,
            width=bar_width,
            label=mode,
            color=palette[mode],
        )
    axes[0, 0].set_title("Mission completion")
    axes[0, 0].set_ylabel("success [%]")
    axes[0, 0].set_ylim(0, 105)
    axes[0, 0].set_xticks(positions, scenarios)
    axes[0, 0].legend(frameon=False, ncol=3, fontsize=8)

    wall_data = [
        data[(data["mode"] == mode) & data["wall_ms"].notna()]["wall_ms"]
        for mode in modes
    ]
    box = axes[0, 1].boxplot(
        wall_data,
        tick_labels=modes,
        patch_artist=True,
        medianprops={"color": "#102a43", "linewidth": 1.5},
    )
    for patch, mode in zip(box["boxes"], modes):
        patch.set_facecolor(palette[mode])
        patch.set_alpha(0.75)
    axes[0, 1].set_title("Planner wall time")
    axes[0, 1].set_ylabel("milliseconds")
    axes[0, 1].tick_params(axis="x", rotation=25)

    for mode in modes:
        subset = data[data["mode"] == mode]
        x_values = np.full(len(subset), modes.index(mode), dtype=float)
        jitter = np.linspace(-0.12, 0.12, max(1, len(subset)))
        axes[1, 0].scatter(
            x_values + jitter,
            subset["margin"],
            s=24,
            alpha=0.8,
            color=palette[mode],
        )
        axes[1, 1].scatter(
            x_values + jitter,
            subset["slack"],
            s=24,
            alpha=0.8,
            color=palette[mode],
        )
    axes[1, 0].axhline(-0.07, color="#c62828", linestyle="--", linewidth=1)
    axes[1, 0].set_title("Worst braking margin per run")
    axes[1, 0].set_ylabel("meters")
    axes[1, 0].set_xticks(range(len(modes)), modes, rotation=25)
    axes[1, 1].axhline(-0.10, color="#c62828", linestyle="--", linewidth=1)
    axes[1, 1].set_title("Worst recovery slack per run")
    axes[1, 1].set_ylabel("seconds")
    axes[1, 1].set_xticks(range(len(modes)), modes, rotation=25)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=280)
    plt.close(figure)


if __name__ == "__main__":
    main()
