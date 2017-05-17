import rospy
import actionlib
import numpy as np

import std_msgs.msg

from trajectory_msgs.msg import *
from control_msgs.msg import *

# Global variables
joint_names = ['j1','j2']
first_time = True

desiredPositions = [[]] * len(joint_names)
actualPositions = [[]] * len(joint_names)
errorPositions = [[]] * len(joint_names)


# Build Trajectory Message
def buildMsg():
    global joint_names

    msg = JointTrajectory()

    msg.joint_names = joint_names

    time = np.linspace(0, 20, 100)

    for t in time:
        traj = JointTrajectoryPoint()
        traj.positions.append(np.sin(t) * 200 + 600)
        traj.positions.append(np.cos(t) * 200 + 600)
        # traj.velocities.append(np.cos(t) * 200 + 600)
        # traj.velocities.append(-np.sin(t) * 200 + 600)

        traj.time_from_start = rospy.Duration(t)
        msg.points.append(traj)

    return msg


def feedbackCallBack(feedback):
    global joint_names, first_time, desiredPositions, actualPositions, errorPositions

    for joint in range(0, len(joint_names)):
        desiredPositions[joint].append(feedback.desired.positions[joint])
        actualPositions[joint].append(feedback.actual.positions[joint])
        errorPositions[joint].append(feedback.error.positions[joint])


def doneCallBack(status, result):
    print 'done'
    print desiredPositions
    print actualPositions
    print errorPositions


if __name__ == '__main__':
    rospy.init_node('jtPub')

    client = actionlib.SimpleActionClient('/joint_trajectory_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction)
    print 'Waiting For Server'
    client.wait_for_server()
    print 'Connected'
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = buildMsg()
    print 'after GOAL'
    client.send_goal(goal, done_cb=doneCallBack, feedback_cb=feedbackCallBack)
    client.wait_for_result()
    result = client.get_result()

    print result
