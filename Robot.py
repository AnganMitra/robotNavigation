import chlotoide as path
import simpy
from parameters import *
import numpy as np
import math
import bspline_path as bp
from scipy.stats import linregress

class Robot(object):
    def __init__(self, env, position, orientation, velocity, People_List, target):
        self.env = env
        self.position = position
        self.orientation = orientation
        self.velocity = velocity
        self.People_List = People_List
        self.process = env.process(self.move())
        self.trajectory = []
        self.target = target
        self.recorded_position_x= [self.position[0]]
        self.recorded_position_y= [self.position[1]]
        self.recorded_velocity = []
        self.recorded_acceleration = []
        self.recorded_orientation =[]
        self.recorded_cost = []

    def get_points_to_target(self):
        slope = np.arctan2( self.target[1]-self.position[1],self.target[0]-self.position[0]  )
        points = [ [self.position[0],self.position[1]]]
        dist = np.linalg.norm(np.array(self.position) - self.target)
        multiplier = 0.4
        for pt in range(8):
            multiplier += 0.1
            slope = slope/(pt+1)
            points.append([self.position[0] + multiplier*dist*np.cos(slope)  , self.position[1] + multiplier*dist*np.sin(slope)])
        # points.append([self.position[0] + 0.6*dist*np.cos(slope/2)  , self.position[1] + 0.6*dist*np.sin(slope/2)])
        # points.append([self.position[0] + 1.2*dist*np.cos(slope/4)  , self.position[1] + 1.2*dist*np.sin(slope/4)])
        return (bp.smoothen(points + [[self.target[0], self.target[1]]]))
        # return  points + [[self.target[0], self.target[1]]]

    def move(self):
        while True:
            self.trajectory, cost, target_approach_bool  = path.getPath(self.People_List, self.position, self.orientation, self.People_List[TARGET_INDEX])
            self.recorded_cost.append(cost)
            self.target= path.compute_target_position(np.array(self.People_List[TARGET_INDEX].position), self.People_List[TARGET_INDEX].orientation, self.position)
            if np.linalg.norm(np.array(self.position) - self.target ) < 0.5 * SAFE_RADIUS and np.linalg.norm(self.People_List[TARGET_INDEX].velocity) < 0.01:
                    self.trajectory = self.get_points_to_target()
                    # print ( " ---------- ",self.target, self.position)
                    # print ("target approaching")
            else:
                self.trajectory = bp.smoothen( [[x,y] for x,y in zip (self.recorded_position_x[-2:], self.recorded_position_y[-2:])]  + self.trajectory)[2:]
            for index in range (1,len(self.trajectory)): # (len(self.trajectory)):
                event = simpy.events.Timeout(self.env, delay=MOTOR_COMMAND_DELAY)
                yield event
                vector = np.array(self.trajectory[index]) - np.array(self.position)
                if vector[0] != 0.0 and np.linalg.norm(vector) > 0.05:
                    self.orientation = math.atan2(vector[1], vector[0])
                # print (self.orientation, np.linalg.norm(vector))
                self.position = self.trajectory[index]
                acceleration = vector/MOTOR_COMMAND_DELAY - self.velocity
                self.velocity = vector/MOTOR_COMMAND_DELAY
                # print ("Robot time ", self.env.now, " position ", self.position, " orientation ", self.orientation, " velocity ", self.velocity)
                self.recorded_position_x.append(self.position[0])
                self.recorded_position_y.append(self.position[1])
                self.recorded_velocity.append(self.velocity)
                self.recorded_acceleration.append(acceleration)
                self.recorded_orientation.append(self.orientation)
