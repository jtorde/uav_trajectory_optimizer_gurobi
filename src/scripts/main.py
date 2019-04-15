#!/usr/bin/python
from solver import *


my_solver=Solver(JERK)
my_solver.setInitialState([0,0,1,0,0,0,0,0,0,0,0,0,0,0,0])
my_solver.setFinalState([0,0,1,0,0,0,0,0,0,0,0,0,0,0,0])
my_solver.setMaxValues([5,3,5,20,40])
my_solver.setN(40)
my_solver.setRadius(0.5)
my_solver.solve();
my_solver.plotSolution();

