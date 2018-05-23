import numpy as np
import dwa
import math
from matplotlib import pyplot as plt
import time

sigma_ps = 5
sigma_gaze = np.pi /4
MOTOR_COMMAND_DELAY = 0.1
SIM_TIME = 50
SAFE_RADIUS = 5

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
        if dist < SAFE_RADIUS:
            cost+= 10
        else:
            cost += np.exp( -dist**2 / sigma_ps**2)
    return cost


# initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
x = np.array([-10.0, 0.0, 0.0, 0.0, 0.0])


# ob = np.matrix([[20.0, 20.0],
#                 [20.0, .0],
#                 [-10.0, 20.0],
#                 [30.0, 0.0],
#                 ])
#
# ob_vel = np.matrix([
#                 [0.0, 0.0],
#                 [-0.04, 0.0],
#                 [0.04, -0.04],
#                 [-0.04, 0.05],
#                 ])
#

# ob = np.matrix([[-20.0, 0.0],
#                 ])
# ob_vel = np.matrix([
#                 [0.0, 0.0],
#                 ])

ob = np.matrix([[20.0, 20.0],
                [20.0, .0],
                ])

ob_vel = np.matrix([
                [0.0, 0.0],
                [-0.4, 0.04],
                ])


traj = dwa.test_main(x, ob, ob_vel,int(SIM_TIME/MOTOR_COMMAND_DELAY))
np.savetxt("dwa_robot.csv", traj, delimiter = ",")
cost = [0.0]
jerk = [0.0]
for index in range(1,len(traj)):
    # print (np.linalg.norm(np.array([traj[index,0], traj[index,1]]) - np.array([traj[index-1,0], traj[index-1,1]])))
    cost.append(cost[-1]+interaction(np.array([traj[index,0], traj[index,1]]), ob, ob_vel) + ps (np.array([traj[index,0], traj[index,1]]), ob, ob_vel))
    assert (cost[index-1] < cost[index])
    angular_acc = (traj[index,-2] - traj[index-1,-2])/MOTOR_COMMAND_DELAY
    linear_acc = (traj[index,-1] - traj[index-1,-1])/MOTOR_COMMAND_DELAY
    jerk.append (jerk[-1] + np.sqrt(linear_acc**2 + angular_acc**2))
    ob += ob_vel


ts = str(int(time.time()))

time = np.linspace(0, SIM_TIME, num = len (jerk))
plt.plot(time, jerk)
plt.xlabel("Time (s)")
plt.ylabel("Jerk")
plt.title("Cummulative Jerk")
plt.savefig("jerk_dwa" + ts + ".png")
# plt.show()

plt.cla()
plt.clf()
plt.plot(time, cost)
plt.xlabel("Time (s)")
plt.ylabel("Cost")
plt.title("Social Cost")
plt.savefig("cost_dwa" + ts + ".png")
# plt.show()


print (" social cost ", cost[-1], " Jerk " ,jerk[-1])
