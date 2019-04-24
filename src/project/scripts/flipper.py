#!/usr/bin/env python

import rospy

import numpy as np
import csv

from itertools import chain

from std_srvs.srv import Trigger, TriggerResponse
from acl_msgs.msg import QuadGoal, ViconState
from geometry_msgs.msg import PoseStamped

# TODO: try to avoid doing this
from solver import *

class Flipper:
    """Flipper"""
    def __init__(self):
        
        self.sub_state = rospy.Subscriber('vicon', ViconState, self.state_cb)

        # trigger the flip
        self.srv_flip = rospy.Service('flip', Trigger, self.flip_cb)

        # outer loop setpoints
        self.pub_goal = rospy.Publisher('goal', QuadGoal, queue_size=1)

        # initialize members
        self.state_msg = ViconState()

        # desired control rate
        self.dc = 0.01


    def flip_cb(self, req):
        
        success = self.generate_flip_trajectory()

        return TriggerResponse(success=success, message='')


    def state_cb(self, msg):
        self.state_msg = msg


    def generate_flip_trajectory(self):

        # start optim at current pose
        x0 = np.zeros((12,))
        x0[0] = self.state_msg.pose.position.x
        x0[1] = self.state_msg.pose.position.y
        x0[2] = self.state_msg.pose.position.z

        xf = np.zeros((12,))
        xf = np.copy(x0)

        s = Solver(JERK)
        s.setInitialState(x0.tolist())
        s.setFinalState(xf.tolist())
        s.setMaxValues([5,3,5,20,40])
        s.setN(40)
        s.setRadius(0.5)
        s.solve()

        #
        # Optimization Results
        #

        csvdata = []

        # how many steps are in an optimization segment
        K = int(s.dt/self.dc)

        n = 0
        while not rospy.is_shutdown() and n<s.N:

            # publish each step at a uniform rate
            rate = rospy.Rate(1.0/self.dc)

            k = 0
            while not rospy.is_shutdown() and k<K:

                p, v, a, j = self.getHighRateGoal(s, n, k)

                goal = QuadGoal()
                goal.header.stamp = rospy.Time.now()
                goal.pos.x,   goal.pos.y,   goal.pos.z   = p
                goal.vel.x,   goal.vel.y,   goal.vel.z   = v
                goal.accel.x, goal.accel.y, goal.accel.z = a
                goal.jerk.x,  goal.jerk.y,  goal.jerk.z  = j
                self.pub_goal.publish(goal)

                csvdata.append(','.join(map(str,chain.from_iterable((p,v,a,j)))))

                # maintain uniform timing (with period dc) for each intra-segment step
                rate.sleep()
                k += 1

            # increment the segment number we are working with
            n += 1

        # Publish final state goal
        goal = QuadGoal()
        goal.header.stamp = rospy.Time.now()
        goal.pos.x, goal.pos.y, goal.pos.z = xf[0:3]
        goal.vel.x, goal.vel.y, goal.vel.z = xf[3:6]
        goal.accel.x, goal.accel.y, goal.accel.z = xf[6:9]
        goal.jerk.x, goal.jerk.y, goal.jerk.z = xf[9:12]
        self.pub_goal.publish(goal)

        with open("fliptraj.csv", "wb") as file:
            for line in csvdata:
                file.write(line)
                file.write('\n')

        return True


    def getHighRateGoal(self, s, n, k):
        # tau \in [0, dt]. Convert step index k to a time tau.
        tau = k*self.dc

        p = tuple(s.getPos(n, tau, ii).getValue() for ii in range(3))
        v = tuple(s.getVel(n, tau, ii).getValue() for ii in range(3))
        a = tuple(s.getAccel(n, tau, ii).getValue() for ii in range(3))
        j = tuple(s.getJerk(n, tau, ii).getValue() for ii in range(3))

        return p, v, a, j


if __name__ == '__main__':
    rospy.init_node('flipper', anonymous=False)

    try:
        obj = Flipper()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
