import numpy as np
import dwa
import math
from matplotlib import pyplot as plt

sigma_ps = 5
sigma_gaze = np.pi /4
MOTOR_COMMAND_DELAY = 0.01
SIM_TIME = 10

def interaction(waypoint, ob, ob_vel ):
    cost = 0
    for (index,people) in enumerate(ob):
        vector = waypoint - people
        angle = math.atan2(vector[0,1],vector[0,0]) - math.atan2(ob_vel[index,1],ob_vel[index,0])
        cost+= np.exp( -(angle**2)/(sigma_gaze**2))
    return cost

def ps(waypoint, ob, ob_vel ):
    cost = 0
    for people in ob:
        dist = np.linalg.norm(people - waypoint)
        if dist < 5:
            cost+= 3
        else:
            cost += np.exp( -dist**2 / sigma_ps**2)
    return cost


# initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
# goal position [x(m), y(m)]
goal = np.array([10, 10])
# obstacles [x(m) y(m), ....]
ob = np.matrix([[-1.0, -1.0],
                [0.0, 2.0],
                # [4.0, 2.0],
                # [5.0, 4.0],
                # [5.0, 5.0],
                # [5.0, 6.0],
                # [5.0, 9.0],
                # [8.0, 9.0],
                # [7.0, 9.0],
                # [12.0, 12.0]
                ])

ob_vel = np.matrix([
                [0.001, 0.00],
                [0, 0.002],
                # [0.0040, 0.002],
                # [0.005, 0.004],
                # [0.005, -0.005],
                # [0.005, 0.006],
                # [-0.005, 0.009],
                # [0.008, 0.009],
                # [0.007, -0.009],
                # [0.0012, 0.0012]
                ])
traj = dwa.test_main(x, goal, ob, ob_vel)
cost = [0.0]
jerk = [0.0]
for index in range(1,len(traj)):
    cost.append(cost[-1]+interaction(np.array([traj[index,0], traj[index,1]]), ob, ob_vel) + ps (np.array([traj[index,0], traj[index,1]]), ob, ob_vel))
    angular_acc = (traj[index,-2] - traj[index-1,-2])/MOTOR_COMMAND_DELAY
    linear_acc = (traj[index,-1] - traj[index-1,-1])/MOTOR_COMMAND_DELAY
    jerk.append (jerk[-1] + np.sqrt(linear_acc**2 + angular_acc**2))
    ob += ob_vel

time = np.linspace(0, SIM_TIME, num = len (jerk))
plt.plot(time, jerk)
plt.xlabel("Time (s)")
plt.ylabel("Jerk")
plt.title("Cummulative Jerk")
plt.savefig("jerk_dwa.png")
plt.show()

plt.cla()
plt.clf()
plt.plot(time, cost)
plt.xlabel("Time (s)")
plt.ylabel("Cost")
plt.title("Social Cost")
plt.savefig("cost_dwa.png")
plt.show()
