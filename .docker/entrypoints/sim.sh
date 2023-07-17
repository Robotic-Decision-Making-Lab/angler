#!/bin/bash

# Add results of ArduSub build
export PATH=$HOME/ardupilot/build/sitl/bin:$PATH

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add alpha models
export GZ_SIM_RESOURCE_PATH=$HOME/ws_angler/src/alpha/alpha_description/meshes:$GZ_SIM_RESOURCE_PATH

# Add blue models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ws_angler/src/blue/blue_description/gazebo/models:$HOME/ws_angler/src/blue/blue_description/gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Build ros_gz for Gazebo Garden
export GZ_VERSION=garden
