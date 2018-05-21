import numpy as np
import seaborn as sns
from matplotlib import pyplot as plt
from matplotlib import animation
from parameters import *
import math

row = 800
column = 600
x_bias = -100
y_bias = -100

def social_cost(people_position, waypoint, personal_space_deviation):
    dist = np.linalg.norm(people_position - waypoint)
    if dist < SAFE_RADIUS:
        return 1
    return np.exp( -dist**2 / personal_space_deviation**2)

def interaction_cost(people_position, waypoint, gaze_direction ,gaze_deviation):
    vector = waypoint - people_position
    angle = math.atan2(vector[1],vector[0]) - gaze_direction
    return np.exp( -(angle**2)/(gaze_deviation**2))



def People_influence(People_List, time,x,y):
    total = 0.0
    waypoint = np.array([x,y])
    for people in People_List:
        people_position = np.array([people.recorded_position_x[time],people.recorded_position_y[time]])
        total+= social_cost(people_position, waypoint, people.personal_space_deviation)
        total+= interaction_cost(people_position, waypoint, people.orientation ,people.gaze_deviation)
    return total

def Robot_influence(robot, time,x,y):
    waypoint = np.array([x,y])
    robot_position = np.array([robot.state[time,0], robot.state[time,1]])
    dist = np.linalg.norm(robot_position - waypoint)
    if dist <2 :
        return 1
    else:
        return 0


def generate_heatmap(robot, People_List):
    heat= []
    time = min(len(robot.state), len(People_List[0].recorded_position_y))
    for index in range(time):
        heat.append(np.zeros(shape=(row,column)))
        for x in range(row):
            for y in range(column):
                # heat[-1][x,y] += Robot_influence(robot, index, x+x_bias,y+y_bias)
                heat[-1][x,y] += People_influence(People_List, index,x+x_bias,y+y_bias)
        xmax, xmin = heat[-1].max(), heat[-1].min()
        heat[-1] = (heat[-1] - xmin)/(xmax - xmin)
    return heat



def HeatMap_Animation(robot, People_List):
    heat = generate_heatmap(robot, People_List)
    fig = plt.figure()
    data = heat[0]
    sns.heatmap(data, xticklabels= False, yticklabels= False)
    def init():
        sns.heatmap(np.zeros((row, column)), xticklabels= False, yticklabels= False)
    def animate(i):
        # print (i)
        plt.clf()
        sns.heatmap(heat[i-1], xticklabels= False, yticklabels= False)

    ani = animation.FuncAnimation(fig, animate, init_func = init, repeat = False)
    ani.save('heatmapy.gif', writer='imagemagick', fps=30 )
    plt.show()
