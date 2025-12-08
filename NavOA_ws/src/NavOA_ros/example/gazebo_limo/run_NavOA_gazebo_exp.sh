#!/bin/bash

# Step 1: Launch the Gazebo environment
gnome-terminal -- bash -c "roslaunch NavOA_ros gazebo_limo_env_complex_20.launch"

# Give some time for Gazebo to start
sleep 20

# Step 2: Launch the NavOA control with obstacle information from laser scan
gnome-terminal -- bash -c "roslaunch NavOA_ros NavOA_gazebo_limo.launch"