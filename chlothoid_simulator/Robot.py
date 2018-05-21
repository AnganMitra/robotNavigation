import chlotoide as path
import simpy
from parameters import *
import numpy as np
import math
import bspline_path as bp
from scipy.stats import linregress

class Robot(object):
    def __init__(self, env, position, orientation, velocity, angular_velocity, linear_acceleration, angular_acceleration,People_List, target):
        self.env = env
        self.People_List = People_List
        self.process = env.process(self.move())
        self.cost = [0.0]
        self.target_orientation = []
        self.target = []
        self.state = np.array([0,0,0,0,0,0,0])
        new_state = np.array([position[0], position[1], orientation, velocity, angular_velocity, linear_acceleration, angular_acceleration ])
        self.state = np.vstack((self.state,new_state))

    def move(self):
        while True:
            trajectory = path.getPath(self.People_List, np.array([self.state[-1,0],self.state[-1,1]]), self.state[-1,2], self.People_List[TARGET_INDEX],self.env)
            for index in range(1,3):
                event = simpy.events.Timeout(self.env, delay=MOTOR_COMMAND_DELAY)
                yield event

                new_state=np.array([0,0,0,0,0,0,0])
                vector = np.array([trajectory.waypoint[index][0],trajectory.waypoint[index][1]]) - np.array([self.state[-1,0],self.state[-1,1]])

                theta = math.atan2(vector[1], vector[0])
                angular_velocity = (theta - self.state[-1,2])/MOTOR_COMMAND_DELAY
                angular_acceleration = (angular_velocity - self.state[-1,4])/MOTOR_COMMAND_DELAY

                velocity = (np.linalg.norm(vector))/MOTOR_COMMAND_DELAY
                linear_acceleration = (velocity- self.state[-1,3])/MOTOR_COMMAND_DELAY
                new_state = np.array([trajectory.waypoint[index][0],trajectory.waypoint[index][1], theta, velocity, angular_velocity, linear_acceleration, angular_acceleration ])

                self.state = np.vstack((self.state,new_state))
                self.target.append(trajectory.target_position)
                self.target_orientation.append(trajectory.target_orientation)
                self.cost.append( self.cost[-1] + trajectory.trajectory_cost[index])
