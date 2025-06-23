#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class EmergencyStopNode:
    def __init__(self):
        rospy.init_node("emergency_stop_node", anonymous=True)
   
        rospy.Subscriber("/emergency_stop",Bool, self.emergency_stop_callback)
        
        rospy.Subscriber("/joint_states",JointState, self.joint_state_callback)
        
        self.joint_command_pub = rospy.Publisher(
            "/position_joint_trajectory_controller/command", 
            JointTrajectory, 
            queue_size=10
        )
        
        self.emergency_stop_active = False
        
        self.current_positions = None
        
        self.enforcement_rate = rospy.Rate(10)  #10 Hz
        
        rospy.loginfo("Emergency Stop Node Initialized. Waiting for emergency stop signal...")

    def joint_state_callback(self,msg):
        self.current_positions = msg.position

    def emergency_stop_callback(self,msg):
        if msg.data:
            rospy.logwarn("Emergency stop activated! Movement is now forbidden.")
            self.emergency_stop_active = True
            self.enforce_stop_position()
        else:
            rospy.loginfo("Emergency stop deactivated. Movement allowed.")
            self.emergency_stop_active = False

    def enforce_stop_position(self):
        if self.current_positions is None:
            rospy.logerr("Cannot enforce emergency stop: Current joint positions are unknown.")
            return
        
        stop_trajectory = JointTrajectory()
        stop_trajectory.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]
        
        point = JointTrajectoryPoint()
        point.positions = list(self.current_positions)
        point.velocities = [0.0] * len(self.current_positions)
        point.time_from_start = rospy.Duration(0.1) 

        stop_trajectory.points.append(point)
        
        while self.emergency_stop_active and not rospy.is_shutdown():
            # Continuously publish the stop command
            self.joint_command_pub.publish(stop_trajectory)
            self.enforcement_rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = EmergencyStopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
