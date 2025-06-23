#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectorySubscriber:
    def __init__(self):
        rospy.init_node('trajectory_subscriber', anonymous=True)

        self.trajectory_sub = rospy.Subscriber(
            '/position_joint_trajectory_controller/command',
            JointTrajectory,
            self.trajectory_callback
        )

        rospy.loginfo("Subscribed to /position_joint_trajectory_controller/command")

    def trajectory_callback(self, msg):
        rospy.loginfo("Received Trajectory Command:")
        rospy.loginfo(f"Joint Names: {msg.joint_names}")
        for idx, point in enumerate(msg.points):
            rospy.loginfo(f"Point {idx + 1}:")
            rospy.loginfo(f" Positions: {point.positions}")
            rospy.loginfo(f"Velocities: {point.velocities}")
            rospy.loginfo(f" Time From Start: {point.time_from_start.secs}s {point.time_from_start.nsecs}ns")

        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TrajectorySubscriber()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logerr("TrajectorySubscriber node interrupted.")
