from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from parameters import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl




count = 0
def Animation(robot, People_List):
    fig, ax  = plt.subplots()
    x = [people.recorded_position_x[0] for people in People_List]
    y = [people.recorded_position_y[0] for people in People_List]
    mat_people, = ax.plot(x,y, 'o', markersize = 10)
    x = [robot.recorded_position_x[0]]
    y = [robot.recorded_position_y[0]]
    mat_robot, = ax.plot(x,y, 'p', markersize = 10)
    x = [People_List[TARGET_INDEX].recorded_position_x[0]]
    y = [People_List[TARGET_INDEX].recorded_position_y[0]]
    mat_target, = ax.plot(x,y, 'D', markersize = 10)
    def animate(i):
        global count
        count +=1
        x = [people.recorded_position_x[count] for people in People_List]
        y = [people.recorded_position_y[count] for people in People_List]
        mat_people.set_data(x,y)
        x = [robot.recorded_position_x[count]]
        y = [robot.recorded_position_y[count]]
        mat_robot.set_data(x,y)
        x = [People_List[TARGET_INDEX].recorded_position_x[count]]
        y = [People_List[TARGET_INDEX].recorded_position_y[count]]
        mat_target.set_data(x,y)

        return mat_people, mat_robot,mat_target,

    ax.axis([0, 100, 0, 100 ])
    ani = animation.FuncAnimation(fig, animate, interval = 100, blit = True)
    ani.save('trajectory.gif', writer='imagemagick', fps=30 )
    plt.show()


def Plot2D(robot, People_List):
    for index, people in enumerate(People_List):
        plt.plot(people.recorded_position_x, people.recorded_position_y, label = "Person "+ str(index))
    plt.plot(robot.recorded_position_x, robot.recorded_position_y, label = "Robot", )
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.savefig("plot2D.png")
    plt.show()


def Plot3D(robot, People_List):
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    t = np.linspace(0, SIM_TIME, num = len (People_List[0].recorded_position_y))
    for index, people in enumerate(People_List):
        x = people.recorded_position_x
        y = people.recorded_position_y
        z = t
        line, = ax.plot(x, y, z, label = 'Person ' + str(index), linewidth = 5)

    x = robot.recorded_position_x
    y = robot.recorded_position_y
    z = t
    line, = ax.plot(x, y, z, label='Robot ',  linewidth = 5 )
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Time')
    plt.show()


def Plot_jerk(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.recorded_acceleration))
    jerk = [np.linalg.norm(robot.recorded_acceleration[0])]
    for acc in robot.recorded_acceleration[1:]:
        jerk.append(jerk[-1] + np.linalg.norm(acc) )
    plt.plot(t,jerk)
    plt.xlabel("Time")
    plt.ylabel("Cumulative Acceleration(m/s2)")
    plt.savefig('acceleration.png')
    plt.show()

def Plot_velocity(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.recorded_velocity))
    plt.plot(t,[np.linalg.norm(vel) for vel in robot.recorded_velocity])
    plt.xlabel("Time")
    plt.ylabel("Velocity(m/s)")
    plt.savefig('velocity.png')
    plt.show()

def Plot_cost(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.recorded_cost))
    plt.plot(t,robot.recorded_cost)
    plt.xlabel("Time")
    plt.ylabel("Cost")
    plt.savefig('Cost.png')
    plt.show()

def Plot_orientation(robot):
    t =  np.linspace(0, SIM_TIME, num = len (robot.recorded_orientation))
    plt.plot(t,robot.recorded_orientation)
    plt.xlabel("Time")
    plt.ylabel("orientation(Theta)")
    plt.savefig('orientation.png')
    plt.show()
