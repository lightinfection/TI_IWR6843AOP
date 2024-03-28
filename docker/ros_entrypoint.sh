#!/bin/bash
set -e

# setup ros environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.sh" ]; then
    . "/opt/ros/$ROS_DISTRO/setup.sh"
fi

if [ -f "/opt/ros/$ROS_DISTRO/install/setup.sh" ]; then
    . "/opt/ros/$ROS_DISTRO/install/setup.sh"
fi

if [ -f "$ROS_WORKSPACE/install/setup.sh" ]; then
    . "$ROS_WORKSPACE/install/setup.sh"
fi

exec "$@"
