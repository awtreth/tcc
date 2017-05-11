
import rospy
import numpy as np
import std_msgs.msg

from trajectory_msgs.msg import *

def buildMsg():
	msg = JointTrajectory()
	
	msg.joint_names = ['j1','j2']
    
	time = np.linspace(0,30,100)
    
	for t in time:
		traj = JointTrajectoryPoint()
		traj.positions.append(np.sin(t)*200+600)
		traj.positions.append(np.cos(t)*200+600)
		traj.time_from_start = rospy.Duration(t)
		msg.points.append(traj)
		

	
	return msg

if __name__ == '__main__': 
	
	rospy.init_node('jtPub')
	
	pub = rospy.Publisher('/joint_trajectory_controller/command', JointTrajectory, latch = True)

	
    
	#traj1.velocities = time*0
	#traj2.velocities = time*0
	#traj1.accelerations = time*0
	#traj2.accelerations = time*0
	#traj1.effort = time*0
	#traj2.effort = time*0
    
	
	
	raw_input('Press ENTER to send message')
	pub.publish(buildMsg());

		
	
