#!/usr/bin/env python
import sys
import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal

class QBHandDriver():
    def __init__(self):

        # rospy.init_node('qbhand_driver')
        rospy.loginfo("Initializing qbhand_driver")

        #!TODO: support multiple hands        
        self.jta = actionlib.SimpleActionClient('/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint action server for qbhand')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action server for qbhand!')
        
        rospy.loginfo('Wating for joint state publisher for qbhand')
        msg = rospy.wait_for_message('/qbhand1/joint_states', JointState)
        if msg is not None:
            rospy.loginfo('Found joint state publisher for qbhand!')
        else:
            rospy.logerr('Could not find joint state publisher for qbhand!')
        
        rospy.Subscriber('/qbhand1/joint_states', JointState, self.joint_state_callback)
        self.is_open, self.is_close = False, False
        self.synergy_joint_pos = 0.0

    def joint_state_callback(self, msg):
        self.joint_state = msg
        self.synergy_joint_pos = msg.position[0]
        if self.synergy_joint_pos < 0.055:
            self.is_open = True
        elif self.synergy_joint_pos > 0.95:
            self.is_close = False
        else:
            self.is_open = False
            self.is_close = False
        

    def open(self, duration=3):
        if self.is_open:
            return

        goal = self.get_qbhand_goal('open', duration)
        self.jta.send_goal(goal)
    
    def close(self, duration=3):
        if self.is_close:
            return
        goal = self.get_qbhand_goal('close', duration)
        self.jta.send_goal(goal)

    def get_qbhand_goal(self, mode='open', duration=5, n_interval=20):
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['qbhand1_synergy_joint']
        if mode == 'open':
            tp = np.linspace(self.synergy_joint_pos, 0.05, n_interval)
        elif mode == 'close':
            tp = np.linspace(self.synergy_joint_pos, 0.95, n_interval)
        else:
            raise NotImplementedError
        tt = np.linspace(0, duration, n_interval)

        for i in range(n_interval):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(float(tt[i]))
            point.positions = [float(tp[i])]
            goal.trajectory.points.append(point)
        return goal


if __name__ == '__main__':
    
    qbhand = QBHandDriver()
    qbhand.close()
    qbhand.open()
    qbhand.close()



        
