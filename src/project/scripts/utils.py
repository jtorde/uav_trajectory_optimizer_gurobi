from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import rospy
from numpy import linalg as LA
from pyquaternion import Quaternion
import numpy as np
import math
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

	#print("All Positions")
	#print(all_pos)

	for i in range (0,len(all_pos),1):
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

		print accel

		if(LA.norm(accel)>0.001 and LA.norm(np.cross(accel, axis_z))>0.0001):
			accel=accel/LA.norm(accel)
			
			#print(accel)
			#print(LA.norm(accel))
			#print("normalized=",accel/LA.norm(accel))
			axis=np.cross(accel, axis_z);
			axis=axis/LA.norm(axis)

			dot=np.dot(accel,axis_z)
			angle=math.acos(dot)
			#print("axis=",axis,"angle=",angle, "accel=",accel)
		        

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

	    #robotMarker.mesh_resource = "package://v4/models/quad.stl";
		robotMarker.mesh_resource = "package://project/models/quad.stl";

		markerArray.markers.append(robotMarker) 

	return markerArray