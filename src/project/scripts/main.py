#!/usr/bin/python
from solver import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import numpy as np
from numpy import linalg as LA
from pyquaternion import Quaternion

markerPub = rospy.Publisher('/robotMarker', MarkerArray, queue_size=10)

print("Waiting for a rosmaster...")
rospy.init_node('stateNode', anonymous=True)
rospy.sleep(0.3)
print("Rosmaster found")

N=40;

my_solver=Solver(JERK)
my_solver.setInitialState([0,0,1,0,0,0,0,0,0,0,0,0,0,0,0])
my_solver.setFinalState([0,1,1,0,0,0,0,0,0,0,0,0,0,0,0])
my_solver.setMaxValues([5,3,5,20,40])
my_solver.setN(N)
my_solver.setRadius(0.5)
my_solver.solve();
my_solver.plotSolution();


# topic = 'visualization_marker_array'
# publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)

# publisher_marker = rospy.Publisher(topic, MarkerArray,queue_size=10)

# rospy.init_node('register')

# rospy.sleep(0.3)
# #rospy.sleep(5)

# markerArray = MarkerArray()

# count=0

# while not rospy.is_shutdown():

#    # ... here I get the data I want to plot into a vector called trans

#    marker = Marker()
#    marker.header.frame_id = "map"
#    marker.type = marker.SPHERE
#    marker.action = marker.ADD
#    marker.id=count
#    marker.scale.x = 0.2
#    marker.scale.y = 0.2
#    marker.scale.z = 0.2
#    marker.color.a = 1.0
#    marker.pose.orientation.w = 1.0
#    marker.pose.position.x = 1
#    marker.pose.position.y = 1
#    marker.pose.position.z = 1

#    # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
#    if(count > 10):
#     markerArray.markers.pop(0)
#     break
#    else:
#     count += 1
#    markerArray.markers.append(marker)


# # Publish the MarkerArray
# publisher.publish(markerArray)
# rospy.spin()


all_pos=my_solver.getAllPos();
all_accel=my_solver.getAllAccel();

markerArray = MarkerArray()

for i in range (0,N,1):

    robotMarker = Marker()
    robotMarker.header.frame_id = "map"
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
    accel=[ all_accel[i][0],  all_accel[i][1],  all_accel[i][2]]
    accel=accel/LA.norm(accel)
    # print(accel)
    # print(LA.norm(accel))
    # print("normalized=",accel/LA.norm(accel))
    axis=np.cross(accel, axis_z);
    axis=axis/LA.norm(axis)
    dot=np.dot(accel,axis_z)
    angle=math.acos(dot)
        

    my_quaternion = Quaternion(axis=axis, angle=-angle)
    print(my_quaternion)

    robotMarker.pose.orientation.x = my_quaternion.elements[1]
    robotMarker.pose.orientation.y = my_quaternion.elements[2]
    robotMarker.pose.orientation.z = my_quaternion.elements[3]
    robotMarker.pose.orientation.w = my_quaternion.elements[0]
    robotMarker.scale.x = 1.0
    robotMarker.scale.y = 1.0
    robotMarker.scale.z = 1.0

    robotMarker.lifetime = rospy.Duration()

    robotMarker.color.r = 0.0
    robotMarker.color.g = 1.0
    robotMarker.color.b = 0.0
    robotMarker.color.a = 0.8

    #robotMarker.mesh_resource = "package://v4/models/quad.stl";
    robotMarker.mesh_resource = "package://project/models/quad.stl";

    markerArray.markers.append(robotMarker)

#print(markerArray)
markerPub.publish(markerArray)

print("Published everything, waiting until killed")
rospy.spin()






   # q = Eigen::AngleAxis<double>(-angle, axis);
    # q.normalize();

    # geometry_msgs::Quaternion q_ros;
    # tf::quaternionEigenToMsg(q, q_ros);
    # marker.pose.orientation = q_ros;



    # double dot = accel.dot(axis_z);
    # double angle = acos(dot);
##########

    # visualization_msgs::Marker marker;
    # marker.header.frame_id = "world";
    # marker.header.stamp = ros::Time();
    # marker.ns = "marker_test";
    # marker.id = i;

    # marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    # marker.action = visualization_msgs::Marker::ADD;

    # marker.pose.position.x = x[i][0];
    # marker.pose.position.y = x[i][1];
    # marker.pose.position.z = x[i][2];

    # Eigen::Quaterniond q;


    # Eigen::Vector3d accel(x[i][6], x[i][7], x[i][8] + GRAV);
    # accel.normalize();

    # Eigen::Vector3d axis;

    # Eigen::Vector3d axis_z(0, 0, 1);
    # axis = accel.cross(axis_z);
    # axis.normalize();

    # double dot = accel.dot(axis_z);
    # double angle = acos(dot);
    # /*    if (i == 8)
    #     {
    #       std::cout << "Accel" << accel << std::endl;
    #       std::cout << "Axis " << axis << std::endl;
    #       std::cout << "Angle " << angle << std::endl;
    #     }*/

    # q = Eigen::AngleAxis<double>(-angle, axis);
    # q.normalize();

    # geometry_msgs::Quaternion q_ros;
    # tf::quaternionEigenToMsg(q, q_ros);
    # marker.pose.orientation = q_ros;
    # marker.scale.x = 1;
    # marker.scale.y = 1;
    # marker.scale.z = 1;
    # if (t_bool[i - 1])
    # {
    #   marker.color.r = 1;
    #   marker.color.g = 0;
    # }
    # else
    # {
    #   marker.color.r = 0;
    #   marker.color.g = 1;
    # }
    # marker.color.b = 0;
    # marker.color.a = 1.0;
    # marker.mesh_resource = "package://v4/models/quad.stl";
    # markerarray.markers.push_back(marker);
