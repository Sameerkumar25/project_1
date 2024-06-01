#!usr/bin/env bash

. opt/ros/humble/setup.bash
colcon build --packages-select cpp_pubsub
exit $?
