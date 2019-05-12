from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import rospy
from numpy import linalg as LA
from pyquaternion import Quaternion
import numpy as np
import math
from matplotlib import cm
from tf.transformations import quaternion_from_euler
import os
GREEN=1
RED=2
BLUE=3

def getMarkerArray(color,all_pos,all_accel):
	markerArray = MarkerArray()
	#print("All accelerations")
	#print(all_accel)

	for i in range (0,len(all_accel),1):
		#print("i=",i)
		print(all_accel[i])
	print ("*****************")
	#print("All Positions")
	#print(all_pos)



	skip=2
	for i in range (0,len(all_pos),skip):
		robotMarker = Marker()
		robotMarker.header.frame_id = "world"
		robotMarker.header.seq = i
		robotMarker.header.stamp    = rospy.get_rostime()
		robotMarker.ns = "robot"
		robotMarker.id = i

		robotMarker.type = robotMarker.MESH_RESOURCE # sphere
	    #robotMarker.type = robotMarker.SPHERE # sphere
		robotMarker.action = robotMarker.ADD
		robotMarker.pose.position.x = all_pos[i][0]
		robotMarker.pose.position.y = all_pos[i][1]
		robotMarker.pose.position.z = all_pos[i][2]


		axis_z=[0,0,1]
		#all_accel gives the total accel of the quad, including gravity // Accel_total =  accel_motors - 9.81

		accel=[ all_accel[i][0],  all_accel[i][1],  all_accel[i][2]+9.81]  #This is the accel produced by the motors

		#print accel
		#print "****************"
		#print "accel_before", accel
		if(LA.norm(accel)>0.001 and LA.norm(np.cross(accel, axis_z))>0.0001):
			accel=accel/LA.norm(accel)
			
			#print(accel)
			#print(LA.norm(accel))
			#print("normalized=",accel/LA.norm(accel))
			axis=np.cross(accel, axis_z);
			axis=axis/LA.norm(axis)

			dot=np.dot(accel,axis_z)
			angle=math.acos(dot)
			#print("accel=",accel,"axis=",axis,"angle=",angle)
		        

			my_quaternion = Quaternion(axis=axis, angle=-angle)

			robotMarker.pose.orientation.x = my_quaternion.elements[1]
			robotMarker.pose.orientation.y = my_quaternion.elements[2]
			robotMarker.pose.orientation.z = my_quaternion.elements[3]
			robotMarker.pose.orientation.w = my_quaternion.elements[0]
		elif (accel[2]<0): #Upside down
			my_quaternion = Quaternion(axis=np.array([1,0,0]), angle=-math.pi)

			robotMarker.pose.orientation.x = my_quaternion.elements[1]
			robotMarker.pose.orientation.y = my_quaternion.elements[2]
			robotMarker.pose.orientation.z = my_quaternion.elements[3]
			robotMarker.pose.orientation.w = my_quaternion.elements[0]

		else: #Hover position
			robotMarker.pose.orientation.x = 0
			robotMarker.pose.orientation.y = 0
			robotMarker.pose.orientation.z = 0
			robotMarker.pose.orientation.w = 1

		robotMarker.scale.x = 1.0
		robotMarker.scale.y = 1.0
		robotMarker.scale.z = 1.0

		robotMarker.lifetime = rospy.Duration()
         
   		if(color==RED):
   			robotMarker.color=ColorRGBA(1, 0, 0, 1)
   		if(color==GREEN):
   			robotMarker.color=ColorRGBA(0,1, 0, 1)
  		if(color==BLUE):
			robotMarker.color=ColorRGBA(0, 0, 1, 1)

		color=cm.jet(int(((i*1.0)/len(all_pos)*256)));
		robotMarker.color=ColorRGBA(color[0], color[1], color[2], color[3])


	    #robotMarker.mesh_resource = "package://v4/models/quad.stl";
		robotMarker.mesh_resource = "package://project/models/quad.stl";

		markerArray.markers.append(robotMarker) 

	return markerArray





def spawnWindowInGazebo(x,y,z,roll,pitch,yaw):
	os.system("rosrun gazebo_ros spawn_model -file `rospack find acl_sim`/urdf/window.urdf -urdf -x " + str(x) + " -y " + str(y) + " -z " + str(z) + " -R " + str(roll) + " -P " + str(pitch) + " -Y " + str(yaw)+ " -model gate")
	pass

def getMarkerWindow(x,y,z,r,p,yaw):

	myMarker = Marker()
	myMarker.header.frame_id = "world"
	myMarker.header.seq = 1
	myMarker.header.stamp    = rospy.get_rostime()
	myMarker.ns = "window"
	myMarker.id = 1
	myMarker.type = myMarker.MESH_RESOURCE # sphere
	   #robotMarker.type = robotMarker.SPHERE # sphere
	myMarker.action = myMarker.ADD
	myMarker.pose.position.x = x
	myMarker.pose.position.y = y
	myMarker.pose.position.z = z
	q = quaternion_from_euler(r, p, yaw)
	myMarker.pose.orientation.x=q[0]
	myMarker.pose.orientation.y=q[1]
	myMarker.pose.orientation.z=q[2]
	myMarker.pose.orientation.w=q[3]
	myMarker.mesh_resource = "package://acl_sim/meshes/other/window_buena.stl";
	myMarker.color=ColorRGBA(0, 1, 0, 1)
	myMarker.scale.x = 5;
	myMarker.scale.y = 5;
	myMarker.scale.z = 6;

	return myMarker