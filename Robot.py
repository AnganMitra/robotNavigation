import chlotoide as path
import simpy
from parameters import *
import numpy as np
import math

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

    def move(self):
        while True:
            self.trajectory = path.getPath(self.People_List, self.position, self.orientation, self.target)

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
