import simpy
import random
import numpy
from parameters import *
import math

class People(object):
    """docstring for [object Object]."""
    def __init__(self, env, id, position, orientation, velocity):
        self.id = id
        self.env = env
        self.position = position
        self.orientation = orientation
        self.velocity = velocity
        self.process = env.process(self.move())
        self.personal_space_deviation = 5
        self.gaze_deviation = numpy.pi/4
        self.recorded_position_x= [self.position[0]]
        self.recorded_position_y= [self.position[1]]
        self.recorded_orientation = [self.orientation]
    def move(self):
        while True:
            event = simpy.events.Timeout(self.env, delay=0.1)
            yield event
            random.seed()
            # self.velocity = numpy.array([random.random(), random.random()])*0.1
            self.position += self.velocity
            self.recorded_position_x.append(self.position[0])
            self.recorded_position_y.append(self.position[1])
            self.recorded_orientation.append(math.atan2(self.velocity[1], self.velocity[0]))
            self.orientation = self.recorded_orientation[-1]
