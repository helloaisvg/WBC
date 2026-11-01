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
    robot_frames = []
    joint_frames = []
    events = []
    cloud = []
    with open(path) as source:
        for line in source:
            frame = json.loads(line)
            kind = frame["type"]
            if kind == "odom":
                odom.append(frame)
            elif kind == "obstacles":
                obstacle_frames.append(frame)
            elif kind == "robot":
                robot_frames.append(frame)
            elif kind == "joints":
                joint_frames.append(frame)
            elif kind == "event":
                events.append(frame)
            elif kind == "static_cloud":
                cloud = frame["points"]
    for values in (odom, obstacle_frames, robot_frames, joint_frames, events):
        values.sort(key=lambda frame: frame["t"])
    return (
        odom,
        obstacle_frames,
        robot_frames,
        joint_frames,
        events,
        np.asarray(cloud, dtype=float),
    )


def project(points, yaw, pitch, vertical_scale=1.0):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.empty((0, 2))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    horizontal = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
    depth = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
    vertical = (
        vertical_scale * points[:, 2] * cos_pitch - depth * sin_pitch
    )
    return np.column_stack((horizontal, vertical))


def robot_frame(pose):
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
        [[math.cos(pose["yaw"]), -math.sin(pose["yaw"])],
         [math.sin(pose["yaw"]), math.cos(pose["yaw"])]]
    )
    return corners @ rotation.T + np.asarray([pose["x"], pose["y"]])


def world_points(points, pose):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.empty((0, 3))
    result = np.asarray(points, dtype=float).copy()
    if result.shape[1] == 2:
        result = np.column_stack((result, np.full(len(result), 0.1)))
    result[:, 0] += pose["x"]
    result[:, 1] += pose["y"]
    return result


def local_points(points, pose, vertical_lift=0.0):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return np.empty((0, 3))
    dx = points[:, 0] - pose["x"]
    dy = points[:, 1] - pose["y"]
    cos_yaw = math.cos(pose["yaw"])
    sin_yaw = math.sin(pose["yaw"])
    result = np.column_stack(
        (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            points[:, 2] + vertical_lift,
        )
    )
    return result


def setup_axis(axis, title):
    axis.set_facecolor("#ffffff")
    axis.tick_params(colors="#334155", labelsize=8)
    for spine in axis.spines.values():
        spine.set_color("#aab6c2")
    axis.grid(color="#cbd5df", linewidth=0.65, alpha=0.65)
    axis.set_title(title, color="#172033", loc="left", fontsize=13)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--step", type=float, default=1.0 / 30.0)
    parser.add_argument(
        "--title",
        default="REMANI-R | full map + articulated arm",
    )
    args = parser.parse_args()

    (
        odom,
        obstacle_frames,
        robot_frames,
        joint_frames,
        events,
        cloud,
    ) = load_trace(args.input)
    if not odom:
        raise RuntimeError("No odometry frames recorded.")
    if not robot_frames:
        raise RuntimeError("No whole-body marker frames recorded.")

    start = odom[0]["t"]
    end = odom[-1]["t"]
    all_series = (odom, obstacle_frames, robot_frames, joint_frames, events)
    for series in all_series:
        for frame in series:
            frame["t"] -= start
    times = np.arange(0.0, end - start + 1.0e-6, args.step)
    odom_times = [frame["t"] for frame in odom]
    obstacle_times = [frame["t"] for frame in obstacle_frames]
    robot_times = [frame["t"] for frame in robot_frames]
    joint_times = [frame["t"] for frame in joint_frames]
    event_times = [frame["t"] for frame in events]

    spawn_times = []
    previous_count = 0
    for frame in obstacle_frames:
        count = len(frame["items"])
        if count > previous_count:
            spawn_times.append(frame["t"])
        previous_count = count

    world_yaw = math.radians(-52.0)
    world_pitch = math.radians(18.0)
    arm_yaw = math.radians(-38.0)
    arm_pitch = math.radians(18.0)

    figure = plt.figure(figsize=(16, 8), facecolor="#ffffff")
    world_axis = figure.add_axes([0.035, 0.10, 0.66, 0.80])
    arm_axis = figure.add_axes([0.73, 0.16, 0.24, 0.64])
    setup_axis(world_axis, "Full-map oblique view")
    setup_axis(arm_axis, "Arm close-up | joint-space state")
    arm_axis.set_xlim(-1.45, 1.45)
    arm_axis.set_ylim(-1.35, 1.55)
    figure.suptitle(args.title, color="#111827", fontsize=16, x=0.02,
                    ha="left")
    cloud_xyz = cloud
    if cloud_xyz.size and cloud_xyz.shape[1] < 3:
        cloud_xyz = np.column_stack(
            (cloud_xyz, np.full(len(cloud_xyz), 0.1))
        )
    if cloud_xyz.size:
        cloud_projected = project(cloud_xyz, world_yaw, world_pitch)
        world_axis.set_xlim(
            cloud_projected[:, 0].min() - 0.5,
            cloud_projected[:, 0].max() + 0.5,
        )
        world_axis.set_ylim(
            cloud_projected[:, 1].min() - 0.6,
            cloud_projected[:, 1].max() + 0.6,
        )
    else:
        world_axis.set_xlim(-6.0, 6.0)
        world_axis.set_ylim(-4.0, 3.0)

    if cloud_xyz.size:
        world_axis.scatter(
            cloud_projected[:, 0],
            cloud_projected[:, 1],
            s=2.2,
            c=cloud_xyz[:, 2],
            cmap="RdYlGn",
            vmin=-0.2,
            vmax=1.9,
            alpha=0.68,
            linewidths=0,
            rasterized=True,
        )
    trail_line, = world_axis.plot([], [], color="#0f8f87", linewidth=2.2)
    arm_world_line, = world_axis.plot(
        [], [], color="#006f9e", linewidth=4.0, marker="o",
        markersize=4.8, markerfacecolor="#ffb000", markeredgewidth=0,
        zorder=7
    )
    base_patch = Polygon(
        np.zeros((4, 2)), closed=True, facecolor="#c98b16",
        edgecolor="#5b3b00", linewidth=1.1, zorder=5
    )
    world_axis.add_patch(base_patch)
    heading_line, = world_axis.plot([], [], color="#5b3b00", linewidth=2.0,
                                    zorder=6)
    obstacle_patches = [
        Circle((0.0, 0.0), 0.35, facecolor="#ff5a36",
               edgecolor="#992200", alpha=0.88, zorder=4)
        for _ in range(8)
    ]
    for patch in obstacle_patches:
        patch.set_visible(False)
        world_axis.add_patch(patch)

    arm_base_patch = Polygon(
        np.asarray([[-0.25, -0.2], [0.25, -0.2],
                    [0.25, 0.2], [-0.25, 0.2]]),
        closed=True, facecolor="#dbe5ed", edgecolor="#334155",
        linewidth=1.0, zorder=2
    )
    arm_axis.add_patch(arm_base_patch)
    arm_line, = arm_axis.plot(
        [], [], color="#006f9e", linewidth=4.5, marker="o",
        markersize=6.5, markerfacecolor="#ffb000", markeredgewidth=0,
        zorder=5
    )
    arm_axis.plot([-1.45, 1.45], [0.0, 0.0], color="#94a3b8",
                  linewidth=0.8, alpha=0.55)
    arm_axis.set_xlabel("lateral projection [m]", color="#334155")
    arm_axis.set_ylabel("vertical projection [m]", color="#334155")
    world_axis.set_xlabel("camera-horizontal [m]", color="#334155")
    world_axis.set_ylabel("camera-vertical [m]", color="#334155")

    time_text = world_axis.text(
        0.015, 0.97, "", transform=world_axis.transAxes,
        color="#172033", fontsize=10, va="top"
    )
    event_text = world_axis.text(
        0.015, 0.90, "", transform=world_axis.transAxes,
        color="#b42318", fontsize=11, va="top", weight="bold"
    )
    joint_text = arm_axis.text(
        0.015, 0.97, "", transform=arm_axis.transAxes,
        color="#17415f", fontsize=8.5, va="top", family="monospace"
    )

    def value_at(frames, frame_times, time_value):
        index = bisect.bisect_right(frame_times, time_value) - 1
        return frames[max(0, index)]

    def update(frame_index):
        time_value = times[frame_index]
        pose = value_at(odom, odom_times, time_value)
        robot = value_at(robot_frames, robot_times, time_value)
        joint = value_at(joint_frames, joint_times, time_value)
        obstacle_frame = value_at(obstacle_frames, obstacle_times, time_value)

        previous = value_at(odom, odom_times, max(0.0, time_value - 10.0))
        trail_world = np.asarray(
            [[frame["x"], frame["y"], 0.05]
             for frame in odom
             if previous["t"] <= frame["t"] <= time_value]
        )
        if trail_world.size:
            trail_line.set_data(
                *project(
                    trail_world,
                    world_yaw,
                    world_pitch,
                ).T
            )

        local_corners = robot_frame(
            {"x": 0.0, "y": 0.0, "yaw": pose["yaw"]}
        )
        chassis_world = np.column_stack(
            (local_corners[:, 0] + pose["x"],
             local_corners[:, 1] + pose["y"],
             np.full(len(local_corners), 0.12))
        )
        base_patch.set_xy(
            project(chassis_world, world_yaw, world_pitch)
        )
        heading = np.asarray([
            [pose["x"], pose["y"], 0.15],
            [pose["x"] + 0.42 * math.cos(pose["yaw"]),
             pose["y"] + 0.42 * math.sin(pose["yaw"]), 0.15],
        ])
        heading_line.set_data(
            *project(heading, world_yaw, world_pitch).T
        )

        markers = sorted(robot["items"], key=lambda item: item["id"])
        if markers and len(markers[0].get("points", [])) > 1:
            marker_world = np.asarray(markers[0]["points"], dtype=float)
        else:
            marker_world = np.asarray(
                [[item["x"], item["y"], item["z"]] for item in markers]
            )
        if marker_world.size:
            projected = project(
                marker_world, world_yaw, world_pitch,
                vertical_scale=1.18
            )
            arm_world_line.set_data(projected[:, 0], projected[:, 1])
            local_markers = local_points(
                marker_world, pose, vertical_lift=0.05
            )
            arm_projected = project(
                local_markers, arm_yaw, arm_pitch, vertical_scale=1.35
            )
            arm_line.set_data(arm_projected[:, 0], arm_projected[:, 1])

        obstacles = obstacle_frame["items"]
        for index, patch in enumerate(obstacle_patches):
            if index >= len(obstacles):
                patch.set_visible(False)
                continue
            obstacle = obstacles[index]
            obstacle_world = np.asarray(
                [[obstacle["x"], obstacle["y"], obstacle.get("z", 0.62)]]
            )
            projected = project(
                obstacle_world,
                world_yaw,
                world_pitch,
                vertical_scale=1.18,
            )[0]
            patch.center = projected
            patch.set_radius(max(0.12, obstacle["r"] * 0.72))
            patch.set_visible(True)

        joint_positions = np.degrees(
            np.asarray(joint.get("positions", []), dtype=float)
        )
        if joint_positions.size:
            first = " ".join(f"{value:6.1f}" for value in joint_positions[:3])
            second = " ".join(f"{value:6.1f}" for value in joint_positions[3:6])
            joint_text.set_text(
                "joint angles [deg]\n"
                f"{first}\n{second}"
            )

        time_text.set_text(
            f"t = {time_value:5.1f} s    base speed = {pose['speed']:.2f} m/s"
        )
        event_index = bisect.bisect_right(event_times, time_value) - 1
        recent_event = ""
        if (event_index >= 0 and
                0.0 <= time_value - event_times[event_index] <= 1.8):
            recent_event = events[event_index]["label"]
        recent_spawn = any(
            0.0 <= time_value - spawn_time <= 1.6
            for spawn_time in spawn_times
        )
        event_text.set_text(
            recent_event or
            ("NEW DYNAMIC OBSTACLE OBSERVED" if recent_spawn else "")
        )
        return (
            trail_line,
            arm_world_line,
            base_patch,
            heading_line,
            arm_line,
            time_text,
            event_text,
            joint_text,
            *obstacle_patches,
        )

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
            bitrate=6000,
            extra_args=["-pix_fmt", "yuv420p"],
        ),
        dpi=120,
    )
    plt.close(figure)


if __name__ == "__main__":
    main()
