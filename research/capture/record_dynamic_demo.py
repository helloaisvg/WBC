#!/usr/bin/env python3

import argparse
import json
import math

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs import point_cloud2
from sensor_msgs.msg import JointState, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def stamp_of(msg):
    if msg.header.stamp.to_sec() > 1.0e-6:
        return msg.header.stamp.to_sec()
    return rospy.Time.now().to_sec()


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--cloud-stride", type=int, default=40)
    args = parser.parse_args()

    output = open(args.output, "w")

    def write(payload):
        output.write(json.dumps(payload, separators=(",", ":")) + "\n")
        output.flush()

    cloud_written = False

    def cloud_callback(msg):
        nonlocal cloud_written
        if cloud_written:
            return
        cloud_written = True
        points = []
        for index, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"))
        ):
            if index % args.cloud_stride == 0:
                points.append([
                    round(point[0], 3),
                    round(point[1], 3),
                    round(point[2], 3),
                ])
        write({"type": "static_cloud", "points": points})

    def odom_callback(msg):
        pose = msg.pose.pose
        write(
            {
                "type": "odom",
                "t": stamp_of(msg),
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": yaw_from_quaternion(pose.orientation),
                "speed": math.hypot(msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y),
            }
        )

    def obstacle_callback(msg):
        obstacles = []
        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            velocity = marker.points[0] if marker.points else None
            obstacles.append(
                {
                    "x": marker.pose.position.x,
                    "y": marker.pose.position.y,
                    "z": marker.pose.position.z,
                    "r": 0.5 * marker.scale.x,
                    "vx": velocity.x if velocity else 0.0,
                    "vy": velocity.y if velocity else 0.0,
                }
            )
        write(
            {
                "type": "obstacles",
                "t": stamp_of(msg.markers[0]) if msg.markers else
                     rospy.Time.now().to_sec(),
                "items": obstacles,
            }
        )

    def joint_callback(msg):
        write(
            {
                "type": "joints",
                "t": stamp_of(msg),
                "positions": [round(value, 5)
                              for value in msg.position],
            }
        )

    def robot_callback(msg):
        markers = []
        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            markers.append(
                {
                    "id": marker.id,
                    "x": round(marker.pose.position.x, 5),
                    "y": round(marker.pose.position.y, 5),
                    "z": round(marker.pose.position.z, 5),
                    "qx": round(marker.pose.orientation.x, 5),
                    "qy": round(marker.pose.orientation.y, 5),
                    "qz": round(marker.pose.orientation.z, 5),
                    "qw": round(marker.pose.orientation.w, 5),
                    "points": [
                        [round(point.x, 5),
                         round(point.y, 5),
                         round(point.z, 5)]
                        for point in marker.points
                    ],
                }
            )
        write(
            {
                "type": "robot",
                "t": stamp_of(msg.markers[0]) if msg.markers else
                     rospy.Time.now().to_sec(),
                "items": markers,
            }
        )

    def log_callback(msg):
        event = None
        if "Recoverability threat" in msg.msg:
            event = "Recovery audit triggered"
        elif "Try to replan a safe trajectory" in msg.msg:
            event = "Runtime replanning"
        elif "Temporal safety repair" in msg.msg:
            event = "Temporal repair applied"
        if event:
            write({"type": "event", "t": stamp_of(msg), "label": event})

    rospy.init_node("remani_dynamic_demo_recorder")
    rospy.Subscriber("/map_generator/global_cloud", PointCloud2,
                     cloud_callback, queue_size=1)
    rospy.Subscriber("/mm/car/odom", Odometry, odom_callback,
                     queue_size=100)
    rospy.Subscriber("/map_generator/dynamic_obstacles", MarkerArray,
                     obstacle_callback, queue_size=100)
    rospy.Subscriber("/mm/mani/joint_state", JointState,
                     joint_callback, queue_size=100)
    rospy.Subscriber("/model_vis/vis_mm", MarkerArray,
                     robot_callback, queue_size=100)
    rospy.Subscriber("/model_vis/vis_mm_skeleton", MarkerArray,
                     robot_callback, queue_size=20)
    rospy.Subscriber("/rosout_agg", Log, log_callback, queue_size=300)
    rospy.spin()
    output.close()


if __name__ == "__main__":
    main()
