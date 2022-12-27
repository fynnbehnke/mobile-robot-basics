#!/bin/bash

# Starts the Gazebo_Sim without GUI, the Rviz Visualization and the Stick-marker pub node
roslaunch behnke_mor2 start_sim.launch &

# Starts the node for control of the turtlebot. Node closes when Goal is reached
roslaunch behnke_mor2 move_to_goal.launch x:=4 y:=-1 theta:=0 --wait
roslaunch behnke_mor2 move_to_goal.launch x:=5 y:=-2 theta:=-1.57
roslaunch behnke_mor2 move_to_goal.launch x:=4 y:=-3 theta:=3.14
roslaunch behnke_mor2 move_to_goal.launch x:=3 y:=-2 theta:=1.57
roslaunch behnke_mor2 move_to_goal.launch x:=0 y:=0 theta:=0
wait