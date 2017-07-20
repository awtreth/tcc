import rospy
import actionlib
import numpy as np

import std_msgs.msg

from trajectory_msgs.msg import *
from control_msgs.msg import *

# Global variables
joint_names = [ 'right_shoulder_swing_joint','right_shoulder_lateral_joint','right_elbow_joint']#,\
                #'right_shoulder_swing_joint','right_shoulder_lateral_joint','right_elbow_joint',\
                #'head_pitch_joint','head_yaw_joint']

def get_position(joint_name, t):
    return {
        'right_shoulder_swing_joint': 0.2*(np.cos(t) - np.sin(2*t))-0.4,
        'right_shoulder_lateral_joint': 0.2*(np.sin(2*t) - np.cos(1.5*t))-0.4,
        'right_elbow_joint': 0.2*(np.sin(2*t) - np.sin(t))-0.4,
    }.get(joint_name, 0)

desiredPositions = [[]] * len(joint_names)
actualPositions = [[]] * len(joint_names)
errorPositions = [[]] * len(joint_names)
desired_timestamps = [[]] * len(joint_names)

# Build Trajectory Message
def buildMsg():
    global joint_names

    msg = JointTrajectory()

    wait_time = 1

    msg.joint_names = joint_names

    time = np.linspace(0, 20, 500)

    for t in time:            
        traj = JointTrajectoryPoint()
        
        for joint in joint_names:        
            traj.positions.append(get_position(joint,t))

        traj.time_from_start = rospy.Duration(t+wait_time)
        msg.points.append(traj)

    return msg


def feedbackCallBack(feedback):
    global joint_names, first_time, desiredPositions, actualPositions, errorPositions

    for joint in range(0, len(joint_names)):
        desiredPositions[joint].append(feedback.desired.positions[joint])
        actualPositions[joint].append(feedback.actual.positions[joint])
        errorPositions[joint].append(feedback.error.positions[joint])
        #~ desired_timestamps[joint].append(feedback.status.goal_id.stamp.secs + feedback.status.goal_id.stamp.nsecs/1e9)


def doneCallBack(status, result):
    print 'done'

if __name__ == '__main__':
    rospy.init_node('control_right_arm')

    #client = actionlib.SimpleActionClient('/posvel_joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client = actionlib.SimpleActionClient('/position_trajectory_controller_right/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    print 'Waiting For Server'
    client.wait_for_server()
    print 'Connected'
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = buildMsg()
    client.send_goal(goal, done_cb=doneCallBack, feedback_cb=feedbackCallBack)
    print 'Goal Sent'
    client.wait_for_result()
    result = client.get_result()

    print result
