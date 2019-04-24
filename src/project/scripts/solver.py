POP=6;
CRACKLE=5;
SNAP=4;
JERK=3;
ACCEL=2;
VEL=1;
POS=0;

from gurobipy import *
import numpy as np
import matplotlib.pyplot as plt
import datetime
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg as LA

def fT(a,b): #Factorial Truncated. Examples: fT(5,2)=5*4*3*2 /// fT(4,4)=4 ///fT(2,0)=0
    result=1;
    for i in range(b,a+1):
        result=result*i
    return result  

class Solver:
    def __init__(self, input): #Input=3 for jerk
        self.inp=input 
        self.m = Model("planning")
        pass
    def setInitialState(self,x0):
        self.x0=x0;
    def setFinalState(self,xf):
        self.xf=xf;
    def setN(self,N):
        self.N=N;
    def setMaxValues(self,max_values):
        self.max_values=max_values;
    def setRadius(self,r):
        self.r=r;
    def solve(self):
        self.dt=0.15 # TODO (line search) 
        
        #Create variables
        self.x=[]
        coeff=["ax","ay","az","bx","by","bz","cx","cy","cz","dx","dy","dz","ex","ey","ez","fx","fy","fz","gx","gy","gz","hx","hy","hz"] # JERK: ax*t^3 + bx*t^2 + cx*t + dx // SNAP: ax*t^4 + bx*t^3 + cx*t^2 + dx +e // And so on
        for t in range (self.N+1):  #Time 0......N
            for i in range ((self.inp+1)*3):  #coefficients         
                self.x=self.x+[self.m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name=coeff[i] + str(t))]
        self.x=np.array(self.x).reshape((self.N+1, (self.inp+1)*3)) 
               
        for t in range (0,self.N-1): #Intervals  t=0,....,N-2
            for i in range (0,3):  #Axis x, y, z
                for state in range (0,self.inp): #[POS, VEL, ACCEL,...,self.in-1]
                    self.m.addConstr(     self.getStateOrInput(state,t,self.dt,i)   ==  self.getStateOrInput(state,t+1,0,i)         )  #Continuity in that state

        #Constraint x0==x_initial
        for i in range (0,3):  #Axis x, y, z
            for state in range (0,self.inp): #[POS, VEL, ACCEL,...,self.in-1]
                self.m.addConstr(  self.getStateOrInput(state,0,0,i)  ==  self.x0[i+3*state]  )     

        #Constraint xT==x_final
        for i in range (0,3):  #Axis x, y, z
            for state in range (0,self.inp): #[POS, VEL, ACCEL,...,self.in-1]
                self.m.addConstr(  self.getStateOrInput(state,self.N-1,self.dt,i)  ==  self.xf[i+3*state]  ) 

        #Input Constraints: v<=vmax, a<=amax, u<=umax
        for t in range (0,self.N-1): #Intervals  t=0,....,N-2
            for i in range (3): #Axis x, y, z
                for state_or_input in range (0,self.inp+1): #[POS, VEL, ACCEL,...,self.in]
                    self.m.addConstr(  self.getStateOrInput(state_or_input,t,self.dt,i)  <=  self.max_values[state_or_input-1]  ) 
                    self.m.addConstr(  self.getStateOrInput(state_or_input,t,self.dt,i)  >=  -self.max_values[state_or_input-1]  ) 

        #Constraints needed to force the flip maneuver
        #Point A
        self.m.addConstr(  self.getPos(int(self.N/4),self.dt,0)    ==  self.x0[0] -self.r      ) 
        self.m.addConstr(  self.getPos(int(self.N/4),self.dt,1)    ==  self.x0[1]     )   
        self.m.addConstr(  self.getPos(int(self.N/4),self.dt,2)    ==  self.x0[2]+self.r ) 
        self.m.addConstr(  self.getAccel(int(self.N/4),self.dt,0 )  >= 0.2   ) # Positive X acceleration 


        #Point B (top of the flip)
        self.m.addConstr(  self.getPos(int(self.N/2),self.dt,0)    ==  self.x0[0]     )   
        self.m.addConstr(  self.getPos(int(self.N/2),self.dt,1)    ==  self.x0[1]     )   
        self.m.addConstr(  self.getPos(int(self.N/2),self.dt,2)    ==  self.x0[2]+2*self.r )   
        self.m.addConstr(  self.getAccel(int(self.N/2),self.dt,2)  <=  -1-9.81   )   #Negative Z acceleration 


        # #Point C
        self.m.addConstr(  self.getPos(int(3*self.N/4),self.dt,0)    ==  self.x0[0] +self.r      ) 
        self.m.addConstr(  self.getPos(int(3*self.N/4),self.dt,1)    ==  self.x0[1]     )   
        self.m.addConstr(  self.getPos(int(3*self.N/4),self.dt,2)    ==  self.x0[2]+self.r ) 
        self.m.addConstr(  self.getAccel(int(3*self.N/4),self.dt,0)  <= -0.2   )    # Negative X acceleration 


        #Compute control cost and set objective
        u=[];
        for t in range (0,self.N):
            u=u+[self.getInput(t,0,0),self.getInput(t,0,1),self.getInput(t,0,2)]
        u=np.array(u).reshape((self.N, 3)) 
        control_cost=[np.dot(u[t,:],u[t,:]) for t in range(self.N)]

        self.m.setObjective(quicksum(control_cost), GRB.MINIMIZE)


        self.m.write("model_new.lp")
        self.m.update()
        self.m.optimize ()
        print("*********************************************************")
        print("Time to solve (ms)=",self.m.runtime*1000)
        print("*********************************************************")

    def plotSolution(self):
        posx=[self.getPos(t,0,0).getValue() for t in range (0,self.N)];
        posy=[self.getPos(t,0,1).getValue() for t in range (0,self.N)];
        posz=[self.getPos(t,0,2).getValue() for t in range (0,self.N)];

        vx=[self.getVel(t,0,0).getValue() for t in range (0,self.N)];
        vy=[self.getVel(t,0,1).getValue() for t in range (0,self.N)];
        vz=[self.getVel(t,0,2).getValue() for t in range (0,self.N)];

        ax=[self.getAccel(t,0,0).getValue() for t in range (0,self.N)];
        ay=[self.getAccel(t,0,1).getValue() for t in range (0,self.N)];
        az=[self.getAccel(t,0,2).getValue() for t in range (0,self.N)];

        time=np.arange(0,self.N*self.dt,self.dt)
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        print("Position X")
        print(posx)
        print("Position Y")
        print(posy)
        print("Position Z")
        print(posz)
        ax.scatter(posx,posy,posz, s=8,color='Red', label='Position',alpha=1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.axis('equal')
        ax.set_xlim([self.x0[0]-2*self.r,self.x0[0]+2*self.r])
        ax.set_ylim([self.x0[1]-1,self.x0[1]+1])
        ax.set_zlim([self.x0[2]-2*self.r,self.x0[2]+2*self.r])
        #plt.show()    

        axes = plt.gca()
        axes.scatter(time,posz, color='Blue', label='Position')
        axes.scatter(time,vx, color='Green', label='Position')
        #axes.scatter(time,posy, color='Green', label='Position')
        plt.grid(True)
        #plt.show()

    def getAllPos(self):
        return [(self.getPos(t,0,0).getValue(),self.getPos(t,0,1).getValue(),self.getPos(t,0,2).getValue()) for t in range (0,self.N)]

    def getAllVel(self):
        return [(self.getVel(t,0,0).getValue(),self.getVel(t,0,1).getValue(),self.getVel(t,0,2).getValue()) for t in range (0,self.N)]

    def getAllAccel(self):
        return [(self.getAccel(t,0,0).getValue(),self.getAccel(t,0,1).getValue(),self.getAccel(t,0,2).getValue()) for t in range (0,self.N)]

    def getAllJerk(self):
        return [(self.getJerk(t,0,0).getValue(),self.getJerk(t,0,1).getValue(),self.getJerk(t,0,2).getValue()) for t in range (0,self.N)]

    def getPos(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(POS,t,tau,ii)

    def getVel(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(VEL,t,tau,ii)

    def getAccel(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(ACCEL,t,tau,ii)

    def getJerk(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(JERK,t,tau,ii)

    def getSnap(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(SNAP,t,tau,ii)

    def getCrackle(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(CRACKLE,t,tau,ii)

    def getPop(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(POP,t,tau,ii)

    #Get the input
    def getInput(self,t,tau,ii): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        return self.getStateOrInput(self.inp,t,tau,ii)

    def getStateOrInput(self,query,t,tau,ii): #query is the state or input (SNAP, JERK,...), t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
        result=0
        for j in range(query,self.inp+1):
            result=result+ (fT(j,j-query+1)*self.x[t,3*(self.inp-j)+ii])*tau**(j-query)
        return result