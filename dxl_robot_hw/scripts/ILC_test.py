import rospy
import actionlib
import numpy as np

import matplotlib.pyplot as plt

from trajectory_msgs.msg import *
from control_msgs.msg import *

# Global parameters
joint = 'joint'

# Global Variables
qd  = []
q   = []
ts  = []

def getReference(t)
    return np.sin(2*t) - np.sin(4*t)

just_started = True

def feedbackCallBack(feedback):
    global joint, desiredPoints, actualPoints, errorPoints, first_time, just_started

    if just_started:
        qref = []
        qd = []
        ts = []
        
    qd = feedback.desired.positions[0]
    qref = feedback.desired.positions[0]
    q = feedback.actual.positions[0]-qref

# Build Trajectory Message
def buildMsg(timestamps, wait_time):
    global joint_names

    msg = JointTrajectory()

    msg.joint_names = [joint] 
    trajectoryPoints = getReference(timestamps)

    for t in time:
        traj = JointTrajectoryPoint()
        traj.positions.append(trajectoryPoints[i])
        traj.time_from_start = rospy.Duration(t+wait_time)
        msg.points.append(traj)

    return msg


if __name__ == '__main__':
    rospy.init_node('jtPub')

    client = actionlib.SimpleActionClient('/position_joint_trajectory_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction)
    print 'Waiting For Server'
    client.wait_for_server()
    print 'Connected'
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = buildMsg(np.linspace(0, 10, 100),2)
    print 'after GOAL'
    client.send_goal(goal, feedback_cb=feedbackCallBack)
    client.wait_for_result()
    result = client.get_result()
    print result

    refPoints = 

    # plt.plot(time, targetTrajectory[0])
    plt.plot(np.linspace(0,8,len(desiredPoints[0].pos)), desiredPoints[0].pos)
    # # plt.plot(actualPoints[0].time, actualPoints[0].pos)
    plt.show()
