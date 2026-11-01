#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool


class GripperCycleWaiter:
    def __init__(self):
        self.saw_close = False
        rospy.Subscriber("/mm/mani/gripper_state", Bool, self.callback,
                         queue_size=10)

    def callback(self, msg):
        if msg.data:
            self.saw_close = True
        elif self.saw_close:
            print("gripper cycle complete", flush=True)
            rospy.signal_shutdown("gripper cycle complete")


def main():
    rospy.init_node("remani_video_completion_wait")
    GripperCycleWaiter()
    rospy.spin()


if __name__ == "__main__":
    main()
