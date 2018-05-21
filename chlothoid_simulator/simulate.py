import simpy
import numpy as np
from parameters import *
from plots import *
import Robot
import People
import time
import turtle_animation as ta
import heatmap as hp
env = simpy.Environment()
# env = simpy.rt.RealtimeEnvironment(factor=2)
##            SCENARIO 1
People_List= [
People.People(env, 1,np.array([20.0,20.0]),0*np.pi,np.array([0.0,0.0])),
People.People(env, 2,np.array([20.0,0.0]),np.pi,np.array([-0.04,0])),
People.People(env, 3,np.array([-10.0,20.0]),np.pi/2,np.array([0.04,-0.04])),
People.People(env, 4,np.array([30.0,0.0]),0.25*np.pi,np.array([-0.04,0.05])),
# People.People(env, 5,np.array([26.0, 36.0]),1.5*np.pi,np.array([0.0,-0.05]))
]

##            SCENARIO 2
# People_List= [
# People.People(env, 1,np.array([-20.0,0.0]),np.pi,np.array([0.0,0.0])),   # clearly maintains a minimum distance
# ]

##            SCENARIO 3
# People_List= [                                                    # take care of gaze
# People.People(env, 1,np.array([20.0,20.0]),0*np.pi,np.array([0.,-0.0])),
# People.People(env, 2,np.array([20.0,0.0]),0.75*np.pi,np.array([-0.4,0.4])),
# ]


robot = Robot.Robot(env, np.array([-10.0,0.0]),0.0,0.0,0.0,0.0,0.0 , People_List, People_List[TARGET_INDEX])
print ("start ", time.time())
env.run(until = SIM_TIME)
print ("end ", time.time())

# Plot3D(robot, People_List)
# Plot_velocity(robot)
# Plot_jerk(robot)
# Plot_cost(robot)
ta.turtle_animate(robot, People_List)
# hp.HeatMap_Animation(robot, People_List)
