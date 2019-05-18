#!/usr/bin/env python

# /****************************************************************************
#  *   Copyright (c) 2019 Parker Lusk and Jesus Tordesillas Torres. All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  *    notice, this list of conditions and the following disclaimer in
#  *    the documentation and/or other materials provided with the
#  *    distribution.
#  * 3. Neither the name of this repo nor the names of its contributors may
#  *    be used to endorse or promote products derived from this software
#  *    without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  *
#  ****************************************************************************/

import rospy

import numpy as np
import csv

from itertools import chain

from std_srvs.srv import Trigger, TriggerResponse
#from acl_msgs.msg import QuadGoal, ViconState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from solver import *
from utils import *

TAKEOFF_ALT = 2.5


class Flipper:
    """Flipper"""
    def __init__(self):
        
        self.sub_state = rospy.Subscriber('vicon', ViconState, self.state_cb)


        self.srv_flip = rospy.Service('window', Trigger, self.window_cb)
        self.srv_flip = rospy.Service('line', Trigger, self.line_cb)
        self.srv_flip = rospy.Service('flip_pitch', Trigger, self.flip_pitch_cb)
        self.srv_flip = rospy.Service('flip', Trigger, self.flip_cb)
        self.srv_flip = rospy.Service('flip_trans', Trigger, self.flip_trans_cb)

        self.srv_takeoff = rospy.Service('takeoff', Trigger, self.takeoff_cb)

        # outer loop setpoints
        self.pub_goal = rospy.Publisher('goal', QuadGoal, queue_size=1)

        self.pub_drone_markers=rospy.Publisher('snapshots', MarkerArray, queue_size=10)
        self.pub_window=rospy.Publisher('window', Marker, queue_size=10)

        # initialize members
        self.state_msg = ViconState()

        # desired control rate
        self.dc = 0.01

        # motor spinup time, seconds
        self.spinup_secs = 0

    def window_cb(self, req):
        success = self.generate_trajectory(WINDOW)
        return TriggerResponse(success=success, message='')

    def line_cb(self, req):
        success = self.generate_trajectory(LINE)
        return TriggerResponse(success=success, message='')

    def flip_trans_cb(self, req):
        
        success = self.generate_trajectory(FLIP_TRANS)
        return TriggerResponse(success=success, message='')

    def flip_pitch_cb(self, req):
        success = self.generate_trajectory(FLIP_PITCH)
        return TriggerResponse(success=success, message='')


    def flip_cb(self, req):
        success = self.generate_trajectory(FLIP)
        return TriggerResponse(success=success, message='')


    def takeoff_cb(self, req):

        ts = rospy.get_time()

        # Wait for motors to spin up
        rospy.sleep(self.spinup_secs)

        goal = QuadGoal()
        goal.header.stamp = rospy.Time.now()
        goal.pos.x,   goal.pos.y,   goal.pos.z   = (self.state_msg.pose.position.x, self.state_msg.pose.position.y, TAKEOFF_ALT)
        goal.vel.x,   goal.vel.y,   goal.vel.z   = (0., 0., 0.)
        goal.accel.x, goal.accel.y, goal.accel.z = (0., 0., 0.)
        goal.jerk.x,  goal.jerk.y,  goal.jerk.z  = (0., 0., 0.)
        goal.xy_mode = goal.z_mode = QuadGoal.MODE_POS

        T=0.02
        increment=0.5
        goal.pos.z=self.state_msg.pose.position.z
        for i in range(100):
            goal.pos.z=min(goal.pos.z+increment,TAKEOFF_ALT);
            rospy.sleep(T)
            self.pub_goal.publish(goal)



        # self.pub_goal.publish(goal)

       

        # for i in range(100):
        #     goal.pos.y=min(goal.pos.y+increment,0);
        #     rospy.sleep(3*T)
        #     self.pub_goal.publish(goal)


        return TriggerResponse(success=True, message='')


    def state_cb(self, msg):
        self.state_msg = msg


    def generate_trajectory(self,type):

        # start optimiz at current pose
        x0 = np.zeros((40,))
        x0[0] = self.state_msg.pose.position.x
        x0[1] = self.state_msg.pose.position.y
        x0[2] = self.state_msg.pose.position.z

        xf = np.zeros((40,))
        xf = np.copy(x0)
        if(type==FLIP_TRANS or type==WINDOW):
            xf[0]=x0[0] + 5;

        if(type==LINE):
            xf[0]=x0[0] + 10;

        s = Solver(JERK)
        s.setTypeTrajectory(type)
        s.setInitialState(x0.tolist())
        s.setFinalState(xf.tolist())

        if(type==FLIP or type==FLIP_TRANS or type==FLIP_PITCH or type==WINDOW):
            x=x0[0]
            y=x0[1]
            z=5.5
            r,p,yaw=0,0,0
            if type==FLIP_TRANS or type==FLIP_PITCH:
                yaw=3.14/2.0;
                x=(x0[0]+xf[0])/2.0
            if type==WINDOW:
                y=x0[1]+2
                yaw=3.14/2.0;
                p=3.14/2.0;
                z=3.5
                x=(x0[0]+xf[0])/2.0

            s.setGate(x,y,z,r,p,yaw)
            #spawnWindowInGazebo(x,y,z,r,p,yaw)

            self.pub_window.publish(getMarkerWindow(x,y,z,r,p,yaw))      
 
        s.setMaxValues([10,90,200,5000,1000000])  #Vel, accel, jerk, snap,...       
        s.setRadius(1)

        solved=False

        for dt in np.linspace(0.1, 4.0, num=50): #Line search on dt
            print "Trying with dt= ",dt
            s.setN(20,dt)
            solved=s.solve()
            if(solved==True):
                break

        if(solved==False):
            print("No solution found after doing line search on dt")
            return False;
            


        # Visualize Markers in RVIZ
        K = int(s.dt/self.dc)

        n = 0
        allPositions=[];
        allAccelerations=[];
        while not rospy.is_shutdown() and n<s.N:

            # publish each step at a uniform rate
            rate = rospy.Rate(1.0/self.dc)

            k = 0
            while not rospy.is_shutdown() and k<K:

                p, v, a, j = self.getHighRateGoal(s, n, k)

                goal = QuadGoal()
                goal.header.stamp = rospy.Time.now()
                allPositions.append(p);
                allAccelerations.append(a)
                k += 1

            # increment the segment number we are working with
            n += 1

        self.pub_drone_markers.publish(getMarkerArray(GREEN,allPositions,allAccelerations))



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
                goal.xy_mode = goal.z_mode = QuadGoal.MODE_POS
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
