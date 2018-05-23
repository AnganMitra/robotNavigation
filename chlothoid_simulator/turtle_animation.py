from turtle import *
import numpy as np
from parameters import *
from scipy.interpolate import spline
count = 1
plot_scale = 5
def turtle_animate(Robot, People_List):
    screen = Screen() # create the screen
    setup( width = 2800, height = 2600, startx = 1, starty = 1)
    span = len(Robot.state)
    recorded_position_x = Robot.state[1:,0]
    recorded_position_y = Robot.state[1:,1]
    orientation = Robot.state[:,2]
    robot = Turtle() # create the first turtle
    robot_shape = Shape('compound')
    poly3 = ((12,12),(12,-12), (-12,-12), (-12,12))
    poly2 = ((0,15),(0,30))
    poly1 = ((0,30),(10,20),(0,45),(-10,20))
    poly4 = ((-10,-5),(-10,5),(-5,5),(-5,-5))
    poly5 = ((10,-5),(10,5),(5,5),(5,-5))

    robot_shape.addcomponent(poly3, "red","blue")
    robot_shape.addcomponent(poly2, "red","blue")
    robot_shape.addcomponent(poly1, "blue","blue")
    robot_shape.addcomponent(poly4, "black","black")
    robot_shape.addcomponent(poly5, "black","black")
    screen.register_shape('robot', shape = robot_shape)
    robot.shape('robot')

    screen.register_shape('robot', shape = robot_shape)

    people = [Turtle() for people in People_List]
    people[TARGET_INDEX].color("red")
    robot.penup()
    robot.setpos(plot_scale*recorded_position_x[0], plot_scale*recorded_position_y[0])
    robot.setheading(orientation[0]*180/np.pi)
    robot.pendown()

    target = Turtle()
    target.color("blue")
    target.penup()
    target.setpos(plot_scale*Robot.target[0][0], plot_scale*Robot.target[0][1])
    target.setheading(Robot.target_orientation[0]*180/np.pi)

    for index in range(len(People_List)):
        people[index].penup()
        people[index].setpos(plot_scale*People_List[index].recorded_position_x[0], plot_scale*People_List[index].recorded_position_y[0])
        people[index].setheading(People_List[index].recorded_orientation[0]*180/np.pi)
        people[index].pendown()


    def move_second(): # the function to move the second turtle
        global count
        if count == len (recorded_position_x)-1:
            print ("Completed!")
            return
        robot.goto(plot_scale*recorded_position_x[count], plot_scale*recorded_position_y[count])
        robot.setheading(orientation[count]*180/np.pi)

        target.goto(plot_scale*Robot.target[count][0], plot_scale*Robot.target[count][1])
        target.setheading(Robot.target_orientation[count]*180/np.pi)

        for index in range(len(People_List)):
            people[index].goto(plot_scale*People_List[index].recorded_position_x[count], plot_scale*People_List[index].recorded_position_y[count])
            people[index].setheading(People_List[index].recorded_orientation[count]*180/np.pi)
        count +=1

        screen.ontimer(move_second) # which sets itself up to be called again

    screen.ontimer(move_second) # set up the initial call to the callback

    mainloop() # start everything running
