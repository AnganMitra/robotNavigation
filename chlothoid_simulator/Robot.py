import chlotoide as path
import simpy
from parameters import *
import numpy as np
import math

class Robot(object):
    def __init__(self, env, position, orientation, velocity, angular_velocity, linear_acceleration, angular_acceleration,People_List, target):
        self.env = env
        self.People_List = People_List
        self.process = env.process(self.move())
        self.target_orientation = []
        self.target = []
        self.state = np.array([0,0,0,0,0,0,0,0,0]) # x y, theta,v,omega,vdot, omegadot,jerk,social_cost
        orientation = math.atan2(target.position[1]- position[1],target.position[0]- position[0])
        new_state = np.array([position[0], position[1], orientation, velocity, angular_velocity, linear_acceleration, angular_acceleration,0,0 ])
        self.state = np.vstack((self.state,new_state))
        self.jerk = [0.0]
    def move(self):
        while True:
            trajectory = path.getPath(self.People_List, np.array([self.state[-1,0],self.state[-1,1]]), self.state[-1,2], self.People_List[TARGET_INDEX],self.env)
            for index in range(1,len(trajectory.waypoint)/2):
                event = simpy.events.Timeout(self.env, delay=MOTOR_COMMAND_DELAY)
                yield event

                vector = np.array([trajectory.waypoint[index][0],trajectory.waypoint[index][1]]) - np.array([self.state[-1,0],self.state[-1,1]])

                theta = math.atan2(vector[1], vector[0])
                angular_velocity = (theta - self.state[-1,2])/MOTOR_COMMAND_DELAY
                angular_acceleration = (angular_velocity - self.state[-1,4])/MOTOR_COMMAND_DELAY

                velocity = (np.linalg.norm(vector))/MOTOR_COMMAND_DELAY
                linear_acceleration = (velocity- self.state[-1,3])/MOTOR_COMMAND_DELAY
                jerk = np.sqrt(linear_acceleration**2+angular_acceleration**2)
                new_state = np.array([trajectory.waypoint[index][0],trajectory.waypoint[index][1], theta, velocity, angular_velocity, linear_acceleration, angular_acceleration,jerk,0 ])

                self.state = np.vstack((self.state,new_state))
                self.target.append(trajectory.target_position)
                self.target_orientation.append(trajectory.target_orientation)
                if np.linalg.norm(self.target[-1] - np.array([trajectory.waypoint[index][0],trajectory.waypoint[index][1]])) < 0.5:
                    print ("goal ")
                    # print (trajectory.length)
                else:
                    self.state[-1,-1] = trajectory.trajectory_cost[index]
                # print (np.linalg.norm(np.array([self.state[index,0], self.state[index,1]]) - np.array([self.state[index-1,0], self.state[index-1,1]])))
