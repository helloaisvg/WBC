#!/usr/bin/env python3

import json
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class CompareDynamicScenario:
    def __init__(self):
        self.update_rate = float(rospy.get_param("~update_rate", 20.0))
        self.radius = float(rospy.get_param("~radius", 0.45))
        self.z = float(rospy.get_param("~z", 0.62))
        self.speed = float(rospy.get_param("~speed", 0.9))
        self.first_trigger_x = float(rospy.get_param("~first_trigger_x", -1.45))
        self.first_cx = float(rospy.get_param("~first_cx", 0.15))
        self.first_center_y = float(rospy.get_param("~first_center_y", 0.65))
        self.first_amplitude = float(rospy.get_param("~first_amplitude", 2.0))
        self.second_enabled = int(rospy.get_param("~second_enabled", 1)) != 0
        self.oscillate = int(rospy.get_param("~oscillate", 1)) != 0
        self.output = rospy.get_param("~output", "/tmp/compare_dynamic.json")
        odom_topic = rospy.get_param("~odom_topic", "/mm/car/odom")
        state_type = rospy.get_param("~state_type", "odometry")

        self.x = -3.0
        self.y = 0.0
        self.first_triggered = False
        self.second_triggered = False
        self.first_start = None
        self.second_start = None
        self.started_at = None
        self.last_odom_at = None

        self.publisher = rospy.Publisher(
            "/map_generator/dynamic_obstacles", MarkerArray, queue_size=10
        )
        if state_type == "moma":
            from fake_moma.msg import MomaState

            rospy.Subscriber(
                odom_topic, MomaState, self.moma_state_callback, queue_size=1
            )
        else:
            rospy.Subscriber(
                odom_topic, Odometry, self.odometry_callback, queue_size=1
            )

    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.update_pose()

    def moma_state_callback(self, msg):
        self.x = msg.chassis_odom.pose.pose.position.x
        self.y = msg.chassis_odom.pose.pose.position.y
        self.update_pose()

    def update_pose(self):
        now = rospy.get_time()
        if self.started_at is None:
            self.started_at = now
        self.last_odom_at = now

        if not self.first_triggered and self.x > self.first_trigger_x:
            self.first_triggered = True
            self.first_start = now
        if (
            self.first_triggered
            and not self.second_triggered
            and self.second_enabled
            and self.x > 2.55
            and self.y > 1.0
        ):
            self.second_triggered = True
            self.second_start = now
        self.write_state()

    def write_state(self):
        record = {
            "x": round(self.x, 6),
            "y": round(self.y, 6),
            "first_triggered": int(self.first_triggered),
            "second_triggered": int(self.second_triggered),
        }
        with open(self.output, "w") as stream:
            json.dump(record, stream)

    def make_marker(self, marker_id, cx, cy, elapsed, phase, amplitude):
        period = 4.0 * amplitude / self.speed
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "dynamic_obstacles"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * self.radius
        marker.scale.y = 2.0 * self.radius
        marker.scale.z = 2.0 * self.radius
        marker.color.r = 1.0
        marker.color.g = 0.18
        marker.color.b = 0.03
        marker.color.a = 0.9

        cycle = (elapsed / period + phase) % 1.0
        if not self.oscillate:
            marker.pose.position.y = cy + amplitude - self.speed * elapsed
            velocity_y = -self.speed
        elif cycle < 0.5:
            marker.pose.position.y = cy + amplitude * (1.0 - 4.0 * cycle)
            velocity_y = -self.speed
        else:
            marker.pose.position.y = cy + amplitude * (4.0 * cycle - 3.0)
            velocity_y = self.speed
        marker.pose.position.x = cx
        marker.pose.position.z = self.z

        velocity = geometry_point(0.0, velocity_y, 0.0)
        marker.points.append(velocity)
        return marker

    def publish(self, event):
        markers = MarkerArray()
        now = rospy.get_time()
        if self.first_start is not None:
            markers.markers.append(
                self.make_marker(
                    0, self.first_cx, self.first_center_y,
                    now - self.first_start, 0.0, self.first_amplitude
                )
            )
        if self.second_start is not None:
            markers.markers.append(
                self.make_marker(
                    1, 1.55, 0.35,
                    now - self.second_start, 0.5, 2.0
                )
            )
        self.publisher.publish(markers)


def geometry_point(x, y, z):
    from geometry_msgs.msg import Point

    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def main():
    rospy.init_node("compare_dynamic_scenario")
    scenario = CompareDynamicScenario()
    rate = rospy.Rate(scenario.update_rate)
    while not rospy.is_shutdown():
        scenario.publish(None)
        rate.sleep()


if __name__ == "__main__":
    main()
