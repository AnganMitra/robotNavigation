from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from parameters import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl




def Plot3D(robot, People_List):
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    t = np.linspace(0, SIM_TIME, num = len (People_List[0].recorded_position_y))
    for index, people in enumerate(People_List):
        x = people.recorded_position_x
        y = people.recorded_position_y
        z = t
        line, = ax.plot(x, y, z, label = 'Person ' + str(index), linewidth = 5)

    x = robot.state[1:,0]
    y = robot.state[1:,1]
    z = t
    line, = ax.plot(x, y, z, label='Robot ',  linewidth = 5 )
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Time')
    plt.show()


def Plot_jerk(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.state))
    jerk = [np.sqrt(robot.state[0,5]**2+robot.state[0,6]**2)]
    for acc in robot.state[1:]:
        jerk.append(jerk[-1] + np.sqrt(acc[5]**2+acc[6]**2 ))
    plt.plot(t,jerk)
    plt.xlabel("Time")
    plt.ylabel("Cumulative Acceleration(m/s2)")
    plt.savefig('acceleration.png')
    plt.show()

def Plot_velocity(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.state))
    plt.plot(t,robot.state[:,3])
    plt.xlabel("Time")
    plt.ylabel("Velocity(m/s)")
    plt.savefig('velocity.png')
    plt.show()

def Plot_cost(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.cost))
    plt.plot(t,robot.cost)
    plt.xlabel("Time")
    plt.ylabel("Cost")
    plt.savefig('Cost.png')
    plt.show()
