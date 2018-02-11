#!/usr/bin/env bash

# Add the plugin path
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/Workspaces/agile_ws/devel/lib
# Add the worlds path
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$HOME/Workspaces/agile_ws/src/agile-gazebo/worlds
# Add the model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/Workspaces/agile_ws/src/agile-gazebo/models