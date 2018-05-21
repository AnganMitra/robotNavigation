from scipy.integrate import quad
import numpy as np
from parameters import *
import matplotlib.pyplot as plt
import math
import bspline_path as bp

length_set = np.linspace(TRAJECTORY_MIN, TRAJECTORY_MAX, num = (TRAJECTORY_MAX - TRAJECTORY_MIN)*10)
def x_integrand(s, params):
    a = params[0]
    b = params[1]
    c = params[2]
    return np.cos(a*s**2 + b*s + c)

def y_integrand(s, params):
    a = params[0]
    b = params[1]
    c = params[2]
    return np.sin(a*s**2 + b*s + c)

def chlotoide_x_y_list(initial_position, length, params):
    field = np.linspace(0, length,TRAJECTORY_PIECES)
    x_chlotoide_coord = [ quad(x_integrand, 0, s, args=params)[0] for s in field]
    y_chlotoide_coord = [ quad(y_integrand, 0, s, args=params)[0] for s in field]
    return x_chlotoide_coord, y_chlotoide_coord

def social_cost(people_position, waypoint, personal_space_deviation):
    dist = np.linalg.norm(people_position - waypoint)
    if dist < 2*SAFE_RADIUS:
        return 10
    return np.exp( -dist**2 / personal_space_deviation**2)

def interaction_cost(people_position, waypoint, gaze_direction ,gaze_deviation):
    vector = waypoint - people_position
    angle = math.atan2(vector[1],vector[0]) - gaze_direction
    return np.exp( -(angle**2)/(gaze_deviation**2))

def heuristic(robot_position, target_position):
    return np.linalg.norm(robot_position - target_position )

def compute_target_position(people_position, gaze_direction, robot_position):
    d = np.sign(robot_position[1] - people_position[0]- np.tan(gaze_direction)*(robot_position[0]-people_position[0]))
    theta = gaze_direction + d*np.pi/6
    return people_position + 1.5*SAFE_RADIUS*np.array([np.cos(theta), np.sin(theta)]), -d*np.pi/6

class Trajectory(object):
    """docstring for [object Object]."""
    def __init__(self, initial_position, length, params, quadrant):
        self.length = length
        self.params = params
        self.waypoint = []
        self.velocity = length/LOOK_AHEAD_TIME
        self.initial_position = initial_position
        self.generate_waypoint(quadrant)
        self.trajectory_cost = []
        self.target_approach_bool = False
        self.target_position = None
        self.target_orientation = None


    def generate_waypoint(self, quadrant):
        x_chlotoide_coord, y_chlotoide_coord= chlotoide_x_y_list(self.initial_position, self.length, self.params)

        if quadrant == 1:
            self.waypoint = [[x+ self.initial_position[0],y+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 2:
            self.waypoint = [[-x+ self.initial_position[0],y+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 3:
            self.waypoint = [[x+ self.initial_position[0],-y+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 4:
            self.waypoint = [[-x+ self.initial_position[0],-y+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 5:
            self.waypoint = [[y+ self.initial_position[0],x+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 6:
            self.waypoint = [[-y+ self.initial_position[0],x+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 7:
            self.waypoint = [[y+ self.initial_position[0],-x+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]
        if quadrant == 8:
            self.waypoint = [[-y+ self.initial_position[0],-x+ self.initial_position[1]] for (x,y) in zip(x_chlotoide_coord, y_chlotoide_coord)]

        assert (len(self.waypoint) == TRAJECTORY_PIECES)
        # self.waypoint = chlotoide_x_y_list(self.initial_position, self.length, self.params)

    def cost(self, People_List, robot_position,theta, target,time):
        trajectory_cost = 0.0
        for people in People_List:
            people_position = np.array(people.position)
            people_velocity = np.array(people.velocity)
            gaze_direction = people.orientation
            waypoint = None
            for waypoint in self.waypoint:
                people_position +=  people_velocity
                s_cost = social_cost(people_position, waypoint, people.personal_space_deviation)
                i_cost = interaction_cost(people_position, waypoint,gaze_direction, people.gaze_deviation)
                self.trajectory_cost.append(s_cost + i_cost)
                trajectory_cost += s_cost_gain*s_cost + i_cost_gain*i_cost

            if people == target:
                self.target_position, self.target_orientation = compute_target_position( people_position, people.orientation,waypoint)
                trajectory_cost += dist_gain*heuristic(waypoint, self.target_position)*time*time
                angle = math.atan2(self.waypoint[-2][1] - self.waypoint[-2][1] ,self.waypoint[-2][0] - self.waypoint[-2][0] ) - self.target_orientation
                trajectory_cost += angle_gain*angle

        return trajectory_cost

    def plot_trajectory(self):
        x =[ item[0] for item in self.waypoint]
        y =[ item[1] for item in self.waypoint]
        plt.plot(x,y)
        plt.show()



def generate_trajectories(robot_position, theta):
    trajectory_set = []
    for length in length_set:
        for param in param_set:
            trajectory_set.append(Trajectory(robot_position,length, param+[theta], 1))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 2))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 3))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 4))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 5))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 6))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 7))
            # trajectory_set.append(Trajectory(robot_position,length, param+[theta], 8))
    return trajectory_set

def getPath(People_List, robot_position, theta, target,env):
    time = env.now
    trajectory_set=generate_trajectories(robot_position, theta)
    cost_set = [trajectory.cost(People_List,robot_position, theta, target,time) for trajectory in trajectory_set]
    selected_trajectory_index = np.argmin(cost_set)
    return trajectory_set[selected_trajectory_index]

def main ():
    for trajectory in generate_trajectories([0,0], 0):
        print (trajectory.length)
        trajectory.plot_trajectory()

if __name__ == "__main__":
    main()
