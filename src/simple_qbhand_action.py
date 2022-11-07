#!/usr/bin/env python
import sys
import rospy
import actionlib
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal



def get_qbhand_goal(mode='open', duration=5, n_interval=20):
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['qbhand1_synergy_joint']
    if mode == 'open':
        tp = np.linspace(0.95, 0.05, n_interval)
    elif mode == 'close':
        tp = np.linspace(0.05, 0.95, n_interval)
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
    if len(sys.argv) != 1:
        args = rospy.myargv(argv=sys.argv)
        mode = args[1] 
        duration = int(args[2])
    else:
        # open or close
        mode = 'open' 
        # total time to execute
        duration = 5

    rospy.init_node('qb_hand_tester')
    jta = actionlib.SimpleActionClient('/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo('Waiting for joint trajectory action')
    jta.wait_for_server()
    rospy.loginfo('Found joint trajectory action!')

    rospy.loginfo('{} for {} s'.format(mode, duration))
    goal = get_qbhand_goal(mode, duration)
    jta.send_goal_and_wait(goal)


        
