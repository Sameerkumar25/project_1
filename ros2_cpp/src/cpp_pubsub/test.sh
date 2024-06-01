#!usr/bin/env bash

. install/setup.bash
ros2 run cpp_pubsub pubsub_test
RETURN_CODE=$?
exit ${RETURN_CODE}
