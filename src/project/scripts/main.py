#!/usr/bin/python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import numpy as np


from solver import *
from utils import *

markerPub = rospy.Publisher('/robotMarker', MarkerArray, queue_size=10)

print("Waiting for a rosmaster...")
rospy.init_node('stateNode', anonymous=True)
rospy.sleep(0.3)
print("Rosmaster found")

N=40;

my_solver=Solver(CRACKLE)
my_solver.setInitialState([0,0,1,0,0,0,0,0,0,0,0,0,0,0,0])
my_solver.setFinalState([0,0,1,0,0,0,0,0,0,0,0,0,0,0,0])
my_solver.setMaxValues([5,3,5,20,40])
my_solver.setN(N)
my_solver.setRadius(0.5)
my_solver.solve();
my_solver.plotSolution();


markerPub.publish(getMarkerArray(GREEN,my_solver.getAllPos(),my_solver.getAllAccel()))


print("Published everything, waiting until killed")
rospy.spin()


