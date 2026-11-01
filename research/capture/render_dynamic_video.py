#!/usr/bin/env python3

import argparse
import bisect
import json
import math

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon


def load_trace(path):
    odom = []
    obstacle_frames = []
    events = []
    cloud = []
    with open(path) as source:
        for line in source:
            frame = json.loads(line)
            if frame["type"] == "odom":
                odom.append(frame)
            elif frame["type"] == "obstacles":
                obstacle_frames.append(frame)
            elif frame["type"] == "static_cloud":
                cloud = frame["points"]
            elif frame["type"] == "event":
                events.append(frame)
    odom.sort(key=lambda frame: frame["t"])
    obstacle_frames.sort(key=lambda frame: frame["t"])
    events.sort(key=lambda frame: frame["t"])
    return odom, obstacle_frames, events, np.asarray(cloud, dtype=float)


def robot_polygon(x, y, yaw):
    length = 0.5
    width = 0.5
    corners = np.asarray(
        [
            [-length / 2.0, -width / 2.0],
            [length / 2.0, -width / 2.0],
            [length / 2.0, width / 2.0],
            [-length / 2.0, width / 2.0],
        ]
    )
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)],
         [math.sin(yaw), math.cos(yaw)]]
    )
    return corners @ rotation.T + np.asarray([x, y])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--step", type=float, default=0.1)
    args = parser.parse_args()

    odom, obstacle_frames, events, cloud = load_trace(args.input)
    if not odom:
        raise RuntimeError("No odometry frames recorded.")

    start_time = odom[0]["t"]
    end_time = odom[-1]["t"]
    times = np.arange(start_time, end_time + 1.0e-6, args.step)
    odom_times = [frame["t"] for frame in odom]
    obstacle_times = [frame["t"] for frame in obstacle_frames]
    event_times = [frame["t"] for frame in events]
    for frame in odom:
        frame["t"] -= start_time
    for frame in obstacle_frames:
        frame["t"] -= start_time
    for frame in events:
        frame["t"] -= start_time
    times -= start_time
    odom_times = [value - start_time for value in odom_times]
    obstacle_times = [value - start_time for value in obstacle_times]
    event_times = [value - start_time for value in event_times]

    spawn_times = []
    previous_count = 0
    for frame in obstacle_frames:
        count = len(frame["items"])
        if count > previous_count:
            spawn_times.append(frame["t"])
        previous_count = count

    figure, axis = plt.subplots(figsize=(12, 6), facecolor="#071019")
    axis.set_facecolor("#071019")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(-10.5, 10.5)
    axis.set_ylim(-4.5, 4.5)
    axis.tick_params(colors="#8aa0b3")
    for spine in axis.spines.values():
        spine.set_color("#23394b")
    axis.grid(color="#122536", linewidth=0.6, alpha=0.7)
    axis.set_xlabel("x [m]", color="#a8bccd")
    axis.set_ylabel("y [m]", color="#a8bccd")
    axis.set_title("REMANI-R  |  simulated dynamic-obstacle execution",
                   color="#f2f6fa", loc="left", fontsize=15)

    if cloud.size:
        axis.scatter(
            cloud[:, 0],
            cloud[:, 1],
            s=1.2,
            c="#3a5368",
            alpha=0.38,
            linewidths=0,
            rasterized=True,
        )

    path_line, = axis.plot([], [], color="#32d6c1", linewidth=2.0, alpha=0.9)
    robot_patch = Polygon(
        robot_polygon(0.0, 0.0, 0.0),
        closed=True,
        facecolor="#f6c453",
        edgecolor="#fff1bb",
        linewidth=1.2,
        zorder=5,
    )
    axis.add_patch(robot_patch)
    direction_line, = axis.plot([], [], color="#fff1bb", linewidth=2.0, zorder=6)
    obstacle_patches = [
        Circle((0, 0), 0.45, facecolor="#ff5a36", edgecolor="#ffd1c7",
               alpha=0.82, zorder=4)
        for _ in range(6)
    ]
    for patch in obstacle_patches:
        patch.set_visible(False)
        axis.add_patch(patch)

    time_text = axis.text(
        0.01, 0.96, "", transform=axis.transAxes,
        color="#d9e6ef", fontsize=10, va="top"
    )
    event_text = axis.text(
        0.01, 0.88, "", transform=axis.transAxes,
        color="#ffb09f", fontsize=11, va="top", weight="bold"
    )

    def odom_at(time_value):
        index = bisect.bisect_left(odom_times, time_value)
        index = min(index, len(odom) - 1)
        return odom[index]

    def obstacles_at(time_value):
        index = bisect.bisect_right(obstacle_times, time_value) - 1
        if index < 0:
            return []
        return obstacle_frames[index]["items"]

    def update(index):
        time_value = times[index]
        pose = odom_at(time_value)
        previous = odom_at(max(0.0, time_value - 8.0))
        trail = np.asarray(
            [[frame["x"], frame["y"]]
             for frame in odom
             if previous["t"] <= frame["t"] <= time_value]
        )
        if trail.size:
            path_line.set_data(trail[:, 0], trail[:, 1])

        polygon = robot_polygon(pose["x"], pose["y"], pose["yaw"])
        robot_patch.set_xy(polygon)
        direction_line.set_data(
            [pose["x"], pose["x"] + 0.42 * math.cos(pose["yaw"])],
            [pose["y"], pose["y"] + 0.42 * math.sin(pose["yaw"])],
        )

        obstacles = obstacles_at(time_value)
        for obstacle_index, patch in enumerate(obstacle_patches):
            if obstacle_index < len(obstacles):
                obstacle = obstacles[obstacle_index]
                patch.center = (obstacle["x"], obstacle["y"])
                patch.set_radius(obstacle["r"])
                patch.set_visible(True)
            else:
                patch.set_visible(False)

        time_text.set_text(
            f"t = {time_value:5.1f} s    base speed = {pose['speed']:.2f} m/s"
        )
        recent_spawn = any(
            0.0 <= time_value - spawn_time <= 1.6
            for spawn_time in spawn_times
        )
        event_index = bisect.bisect_right(event_times, time_value) - 1
        recent_event = ""
        if (event_index >= 0 and
                0.0 <= time_value - event_times[event_index] <= 1.8):
            recent_event = events[event_index]["label"]
        event_text.set_text(
            recent_event or
            ("NEW DYNAMIC OBSTACLE OBSERVED" if recent_spawn else "")
        )
        return [path_line, robot_patch, direction_line, time_text, event_text,
                *obstacle_patches]

    matplotlib.rcParams["animation.ffmpeg_path"] = "/opt/homebrew/bin/ffmpeg"
    video = animation.FuncAnimation(
        figure,
        update,
        frames=len(times),
        interval=1000.0 / args.fps,
        blit=False,
    )
    video.save(
        args.output,
        writer=animation.FFMpegWriter(
            fps=args.fps,
            bitrate=5000,
            extra_args=["-pix_fmt", "yuv420p"],
        ),
        dpi=140,
    )
    plt.close(figure)


if __name__ == "__main__":
    main()
