#!/bin/bash

gnome-terminal --tab -- bash -c "roslaunch exploration_manager env_simulation.launch ; exec bash"
sleep 10s

gnome-terminal --tab -- bash -c "roslaunch exploration_manager uav_simulation.launch ; exec bash"
sleep 3s

gnome-terminal --tab -- bash -c "roslaunch exploration_manager fuae.launch ; exec bash"

