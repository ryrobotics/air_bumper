#!/bin/bash

# move to the project path
cd ~/catkin_ws/src/air_bumper/scripts/$1

tmuxinator start -p ./.tmuxinator.yml