#!/usr/bin/python

# Authors: Parker Lusk, Jesus Tordesillas, April-May 2019

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


#N_vector=np.linspace(4, 23, num=21)

N=40
tune=0

x0=[0,0,1,0,0,0,0,0,0] #Initial state  x y z vx vy vz ax ay az
xf=[0,0,1,0,0,0,0,0,0] #Final state

print("N=",N)  #N is the number of INTERVALS
dt=0.1 #TODO: Line search on dt
umax=5 #jerk max
amax=3
vmax=5
q=20;

m = Model("planning")

x=[]
coeff=["ax","ay","az","bx","by","bz","cx","cy","cz","dx","dy","dz"] # ax*t^3 + bx*t^2 + cx*t + dx
for t in range (N+1):  #Time 0......20
    for i in range (12):  #coefficients
        # if t==N:
    #       x=x+[m.addVar(lb=-(xf[i]+0.2), ub=(xf[i]+0.2), name=states[i] + str(t))] #Constraint xf \in RegionXFinal
        # else:         
        x=x+[m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name=coeff[i] + str(t))]

#first row of x is: ["ax","ay","az","bx","by","bz","cx","cy","cz","dx","dy","dz"]


x=np.array(x).reshape((N+1, 12))  

#update is needed if you want to print the variables/constraints... (if not it says waiting for update)
m.update()
#print(x)
# print(u)

#matmul doesn't work. should use dot instead
def getPos(t,tau,ii, solved=False): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
    if solved==True:
        pos=(x[t,0+ii].x)*tau**3    +  (x[t,3+ii].x)*tau**2  +  (x[t,6+ii].x)*tau  +  (x[t,9+ii].x)
        return pos
    else:
        pos=x[t,0+ii]*tau**3    +  x[t,3+ii]*tau**2  +  x[t,6+ii]*tau  +  x[t,9+ii]
        return pos

def getVel(t,tau,ii, solved=False): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
    if solved==True:
        vel=(3*x[t,0+ii].x)*tau**2  +  (2*x[t,3+ii].x)*tau   +  (x[t,6+ii].x)
        return vel
    else:
        vel=3*x[t,0+ii]*tau**2  +  2*x[t,3+ii]*tau   +  x[t,6+ii]
        return vel

def getAccel(t,tau,ii, solved=False): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
    if solved==True:
        accel=(6*x[t,0+ii].x)*tau +  2*x[t,3+ii].x
        return accel
    else:
        accel=6*x[t,0+ii]*tau +  2*x[t,3+ii]
        return accel

def getJerk(t,tau,ii, solved=False): #t is the segment, tau is the time inside a specific segment (\in[0,dt], i is the axis)
    if solved==True:
        jerk=6*x[t,0+ii].x  #Note that here tau doesn't appear (makes sense)
        return jerk
    else:
        jerk=6*x[t,0+ii]  #Note that here tau doesn't appear (makes sense)
        return jerk

# print("Position at the beginning")
# print("In X:")
# print(getPos(0,0,0))
# print("In Y:")
# print(getPos(0,0,1))
# print("In Z:")
# print(getPos(0,0,2))

#Continuity constraints xt+1==A*xt+B*ut
for t in range (0,N-1): #Intervals  t=0,....,N-2
    for i in range (0,3):  #Axis x, y, z
        m.addConstr(  getPos(t,dt,i)   ==  getPos(t+1,0,i)     )    #Continuity in position
        m.addConstr(  getVel(t,dt,i)   ==  getVel(t+1,0,i)     )    #Continuity in velocity
        m.addConstr(  getAccel(t,dt,i) ==  getAccel(t+1,0,i)   )    #Continuity in acceleration

#Constraint x0==x_initial
for i in range (0,3):  #Axis x, y, z
    m.addConstr(  getPos(0,0,i)      ==  x0[i]  )         #Initial position
    m.addConstr(  getVel(0,0,i)      ==  x0[i+3]   )      #Initial velocity
    m.addConstr(  getAccel(0,0,i)    ==  x0[i+6]  )       #Initial acceleration

#Constraint xT==x_final
for i in range (0,3):  #Axis x, y, z
    m.addConstr(  getPos(N-1,dt,i)    ==  xf[i]      )   #Final position
    m.addConstr(  getVel(N-1,dt,i)    ==  xf[i+3]    )   #Final velocity
    m.addConstr(  getAccel(N-1,dt,i)  ==  xf[i+6]    )   #Final acceleration

#Input Constraints: v<=vmax, a<=amax, u<=umax
for t in range (0,N-1): #Intervals  t=0,....,N-2
    for i in range (3): #inputs variables

        m.addConstr(  getVel(t,dt,i) <=  vmax  )
        m.addConstr(  getVel(t,dt,i) >=  -vmax  )

        m.addConstr(  getAccel(t,dt,i) <=  amax  )
        m.addConstr(  getAccel(t,dt,i) >=  -amax  )

        m.addConstr(  getJerk(t,dt,i) <=  umax  )
        m.addConstr(  getJerk(t,dt,i) >=  -umax  )


#Constraints needed to force the flip maneuver
r=0.5
#Point A
m.addConstr(  getPos(int(N/4),dt,0)    ==  x0[0] -r      ) 
m.addConstr(  getPos(int(N/4),dt,1)    ==  x0[1]     )   
m.addConstr(  getPos(int(N/4),dt,2)    ==  x0[2]+r ) 
m.addConstr(  getAccel(int(N/4),dt,0 )  >= 0.2   ) # Positive X acceleration 


#Point B (top of the flip)
m.addConstr(  getPos(int(N/2),dt,0)    ==  x0[0]     )   
m.addConstr(  getPos(int(N/2),dt,1)    ==  x0[1]     )   
m.addConstr(  getPos(int(N/2),dt,2)    ==  x0[2]+2*r )   
m.addConstr(  getAccel(int(N/2),dt,2)  <=  -0.2   )   #Negative Z acceleration 


#Point C
m.addConstr(  getPos(int(3*N/4),dt,0)    ==  x0[0] +r      ) 
m.addConstr(  getPos(int(3*N/4),dt,1)    ==  x0[1]     )   
m.addConstr(  getPos(int(3*N/4),dt,2)    ==  x0[2]+r ) 
m.addConstr(  getAccel(int(3*N/4),dt,0)  <= -0.2   )    # Negative X acceleration 


u=[];
for t in range (0,N):
    u=u+[getJerk(t,0,0),getJerk(t,0,1),getJerk(t,0,2)]

u=np.array(u).reshape((N, 3)) 

xf=np.array(xf)


control_cost=[np.dot(u[t,:],u[t,:]) for t in range(N)]

m.setObjective(quicksum(control_cost), GRB.MINIMIZE)


#WRITE HERE THE parameters obtained by the tunning tool (they are written in the file tune.prm)
# m.Params.MIPFocus =1
# m.Params.Heuristics =  0
# m.Params.Cuts = 0
# m.Params.PrePasses= 5
### m.Params.MIPGap=100  Be careful with this parameter. If too high--> weird trajectories

#model.lp is useful to check if the model makes sense
m.write("model.lp")
m.update()



before= datetime.datetime.now();
#m.Params.timeLimit=0.04
#m.Params.solutionLimit=1 #This option saves ~10 ms
m.optimize ()
print("*********************************************************")
print("Time to solve (ms)=",m.runtime)
print("*********************************************************")

before= datetime.datetime.now();


if 1:

    # print("Solved Opt Problem")
    # print("Initial Pos Y")
    # print(getPos(0,0,1,True))
    # print("******")
    posx=[getPos(t,0,0,True) for t in range (0,N)];
    posy=[getPos(t,0,1,True) for t in range (0,N)];
    posz=[getPos(t,0,2,True) for t in range (0,N)];

    vx=[getVel(t,0,0,True) for t in range (0,N)];
    vy=[getVel(t,0,1,True) for t in range (0,N)];
    vz=[getVel(t,0,2,True) for t in range (0,N)];

    ax=[getAccel(t,0,0,True) for t in range (0,N)];
    ay=[getAccel(t,0,1,True) for t in range (0,N)];
    az=[getAccel(t,0,2,True) for t in range (0,N)];

    time=np.arange(0,N*dt,dt)
    
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
    ax.set_xlim([x0[0]-2*r,x0[0]+2*r])
    ax.set_ylim([x0[1]-1,x0[1]+1])
    ax.set_zlim([x0[2]-2*r,x0[2]+2*r])
    plt.show()    

    axes = plt.gca()
    axes.scatter(time,posz, color='Blue', label='Position')
    axes.scatter(time,vx, color='Green', label='Position')
    #axes.scatter(time,posy, color='Green', label='Position')
    plt.grid(True)
    plt.show()
else:
    print("No solution")

# fig = plt.figure()
# axes = plt.gca()
# axes.scatter(N_vector,runtime_vector, color='Green', label='Position')
# plt.xlabel("Number of points")
# plt.ylabel("Runtime (ms)")
# plt.show()




### OLD STUFF


# if tune==1: #Tune parameters if you want to optimize the runtime
#     m.Params.tuneResults = 1
#     m.setParam("TuneTimeLimit", 20.0); #in seconds
#     m.tune()
#     if m.tuneResultCount>0:
#         # Load the best tuned parameters into the model
#         m.getTuneResult(0)
#         # Write tuned parameters to a file
#         m.write('tune.prm')
#         # Solve the model using the tuned parameters
#         before= datetime.datetime.now();
#         m.optimize ()
#         after= datetime.datetime.now();
#         print("*********************************************************")
#         print("Time to solve (ms)=",(after-before).total_seconds() * 1000)
#         print("*********************************************************")
# else:


#Constraint in the first polytope

# A1=np.array([[-0.0990887,    0.994031,  -0.0456529         ],  \
#            [   -0.11874,    0.992494,   0.0292636         ],  \
#            [   0.315838,    0.947835,  -0.0430724         ],  \
#            [   0.279625,    0.956348,   0.0849006         ],  \
#            [   0.0379941,   -0.999235, -0.00925698        ],  \
#            [   0.031154,   -0.999406,   0.0147274         ],  \
#            [          0,          -1,           0         ],  \
#            [         -0,           1,          -0         ],  \
#            [    0.95448,           0,    0.298275         ],  \
#            [   -0.95448,          -0,   -0.298275         ],  \
#            [   0.298275,           0,    -0.95448         ],  \
#            [  -0.298275,          -0,     0.95448         ]])


# b1=np.array([[    11.2113   ],  \
#             [    11.1351   ],  \
#             [    15.1733   ],  \
#             [    15.1708   ],  \
#             [    -9.26336  ],  \
#             [   -9.27607   ],  \
#             [   -9.5       ],  \
#             [   13.5       ],  \
#             [   14.3031    ],  \
#             [   -3.92154   ],  \
#             [    2.01413   ],  \
#             [   -0.014135  ]])
 

# A2=np.array([[ -0.199658,   0.976518,  0.0809297  ],  \
# [-0.166117,   0.983608, -0.0701482],  \
# [-0.358298,  -0.933434,  0.0179951],  \
# [-0.365568,  -0.928824, -0.0603848],  \
# [-0.707107,  -0.707107,          0],  \
# [ 0.707107,   0.707107,         -0],  \
# [ 0.485071,  -0.485071,  -0.727607],  \
# [-0.485071,   0.485071,   0.727607],  \
# [-0.514496,   0.514496,  -0.685994],  \
# [ 0.514496 , -0.514496,   0.685994],  \
# ])
 
 
# b2=np.array([[ 9.06362],  \
# [ 9.21024],  \
# [-13.5612],  \
# [-13.7295],  \
# [-15.3241],  \
# [ 19.3241],  \
# [ 1.60634],  \
# [ 2.45521],  \
# [-1.82973],  \
# [ 3.82973]])


# A3=np.array([[-0.999958, 0.00342454, 0.00852636 ],  \
#  [-0.999832, 0.00363672, -0.0179664],  \
#  [-0.999778, -0.0204566, 0.00504416],  \
#  [-0.999564, -0.0227306, -0.0188383],  \
#  [       -1,         -0,          0],  \
#  [        1,          0,         -0],  \
#  [        0,   -0.98387,  0.178885],  \
#  [       -0,    0.98387,  -0.178885],  \
#  [        0,  -0.178885,   -0.98387],  \
#  [       -0,   0.178885,    0.98387]])

# b3=np.array([[  -12.7365  ],  \
#             [   -12.7834 ],  \
#             [   -12.9236 ],  \
#             [   -12.9824 ],  \
#             [        -12 ],  \
#             [         16 ],  \
#             [   -3.47214 ],  \
#             [    11.0623 ],  \
#             [    -2.3541 ],  \
#             [     4.3541 ]])

#I think in Gurobi you can't add a constraint like vector<=vector. It has to be element wise
# n1=12
# n2=12
# for t in range (0,n1): 
#     for i in range(0,b1.size): 
#         m.addConstr(np.dot(A1,x[t][0:3])[i]<=b1[i])

# for t in range (n1,n2): 
#     for i in range(0,b2.size): 
#         m.addConstr(np.dot(A2,x[t][0:3])[i]<=b2[i])

# for t in range (n2,N): 
#     for i in range(0,b3.size): 
#         m.addConstr(np.dot(A3,x[t][0:3])[i]<=b3[i])