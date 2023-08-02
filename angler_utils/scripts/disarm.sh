#!/usr/bin/env bash

# This is a simple script used to make your life easier when disarming the full system

ros2 service call /blue/cmd/enable_passthrough std_srvs/srv/SetBool data:\ false\

ros2 service call /blue/cmd/arm std_srvs/srv/SetBool data:\ false\

ros2 service call /angler/tpik/arm std_srvs/srv/SetBool data:\ false\
