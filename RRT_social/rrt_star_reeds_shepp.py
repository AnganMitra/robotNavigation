import sys
sys.path.append("./ReedsSheppPath/")

import random
import math
import copy
import numpy as np
import reeds_shepp_path_planning
import matplotlib.pyplot as plt

show_animation = False
STEP_SIZE = 0.6
curvature = 1.0


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 goalSampleRate=10, maxIter=400):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            if newNode is None:
                continue

            if self.CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                if newNode is None:
                    continue
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if tNode is None:
                continue

            if self.CollisionCheck(tNode, self.obstacleList):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            # print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)

        return newNode

    def pi_2_pi(self, angle):
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def steer(self, rnd, nind):

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature, STEP_SIZE)

        if px is None:
            return None

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += sum([abs(c) for c in clen])
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self):
        #  print("get_best_last_index")

        YAWTH = math.radians(3.0)
        XYTH = 0.5

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)
        #  print("OK XY TH num is")
        #  print(len(goalinds))

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)
        #  print("OK YAW TH num is")
        #  print(len(fgoalinds))

        if len(fgoalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):

        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)
            if tNode is None:
                continue

            obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

        #  plt.show()
        #  input()

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node, obstacleList):

        for (ox, oy) in obstacleList:
            size = 1
            for (ix, iy) in zip(node.path_x, node.path_y):
                dx = ox - ix
                dy = oy - iy
                d = dx * dx + dy * dy
                if d <= size ** 2:
                    return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None

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

def compute_target_position(people_position, gaze_direction, robot_position):
    d = np.sign(robot_position[1] - people_position[0]- np.tan(gaze_direction)*(robot_position[0]-people_position[0]))
    theta = gaze_direction + d*np.pi/6
    return people_position + 1.5*SAFE_RADIUS*np.array([np.cos(theta), np.sin(theta)]) , -d*np.pi/6

def main(iteration):
    print("Start rrt start planning")

    ob = np.matrix([[20.0, 20.0],
                    [20.0, .0],
                    ])

    ob_vel = np.matrix([
                    [0.0, 0.0],
                    [-0.4, 0.04],
                    ])
    # Set Initial parameters
    state = np.array([0,0,0,0,0,0,0,0,0])
    state = np.vstack((state,state))
    start = [0.0, 0.0, math.radians(0.0)]
    tp, to = compute_target_position(np.array([ob[0,0], ob[0,1]]), math.atan2(ob_vel[0,1], ob_vel[0,0]), np.array([state[-1,0], state[-1,1]]))
    goal = [tp[0], tp[1], math.radians(to)]

    for i in range(iteration):
        print (i)
        rrt = RRT(start, goal, randArea=[-20.0, 30.0], obstacleList=ob.tolist())
        path = rrt.Planning(animation=show_animation)
        path = np.flipud(path)
        for j in range (len(path)/2):
            vector = np.array([path[j][0],path[j][1]]) - np.array([state[-1,0],state[-1,1]])

            theta = math.atan2(vector[1], vector[0])
            angular_velocity = (theta - state[-1,2])/MOTOR_COMMAND_DELAY
            angular_acceleration = (angular_velocity - state[-1,4])/MOTOR_COMMAND_DELAY

            velocity = (np.linalg.norm(vector))/MOTOR_COMMAND_DELAY
            linear_acceleration = (velocity- state[-1,3])/MOTOR_COMMAND_DELAY
            jerk = state[-1, -2] + np.sqrt(linear_acceleration**2+angular_acceleration**2)
            cost = state[-1, -1] + interaction(np.array([path[j][0], path[j][1]]), ob, ob_vel) + ps (np.array([path[j][0], path[j][1]]), ob, ob_vel)
            new_state = np.array([path[j][0], path[j][1], theta, velocity, angular_velocity, linear_acceleration, angular_acceleration,jerk,cost ])
            state = np.vstack((state, new_state))
            ob += ob_vel
        start = [state[-1,0], state[-1,1], math.radians(state[-1,2])]
        tp, to = compute_target_position(np.array([ob[0,0], ob[0,1]]), math.atan2(ob_vel[0,1], ob_vel[0,0]), np.array([state[-1,0], state[-1,1]]))
        goal = [tp[0], tp[1], math.radians(to)]
        # Draw final path
        # if show_animation:
        #     rrt.DrawGraph()
        #     plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        #     plt.grid(True)
        #     plt.pause(0.001)
        #     plt.show()
    print (" social cost ", state[-1, -1], " Jerk " ,state[-1, -2])
    np.savetxt("./results/rrt_social.csv", state, delimiter = ",")
if __name__ == '__main__':
    main(10)
