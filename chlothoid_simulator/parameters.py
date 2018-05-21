SIM_TIME = 50
TRAJECTORY_MIN = 0
TRAJECTORY_MAX = 2.5
LOOK_AHEAD_TIME = 1
TRAJECTORY_PIECES = 20
MOTOR_COMMAND_DELAY = 0.1
param_set =[
[0.01,0.01],
[0.02,0.02],
[0.03,0.03],
[0.04,0.04],
[0.05,0.05],
[0.0, 0.05],
[1,1],
[0.0, 0,0],
[-0.01,-0.01],
[-0.02,-0.02],
[-0.03,-0.03],
[-0.04,-0.04],
[-0.05,-0.05],
[0.0, -0.05],
]

TARGET_INDEX = 0
SAFE_RADIUS = 5
s_cost_gain = 10.0
i_cost_gain = 1.5
angle_gain =  1.0
dist_gain = 10.0
