#!/bin/env python

robot_saved_state = open('/home/ubuntu/ROS/arlo_ws/src/arlo-motor-controller-interface/saved-robot-state')

robot_saved_state.readline()
robot_saved_state.readline()
print(robot_saved_state.readline().split()[1])
print(robot_saved_state.readline().split()[1])
print(robot_saved_state.readline().split()[1])
robot_saved_state.readline()
print(robot_saved_state.readline().split()[1])
print(robot_saved_state.readline().split()[1])
print(robot_saved_state.readline().split()[1])
print(robot_saved_state.readline().split()[1])

print(eval(robot_saved_state.readline().split(': ')[1]))




