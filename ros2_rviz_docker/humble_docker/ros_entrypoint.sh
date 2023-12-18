#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
 
### ROS_MASTER_URI: ip of the robot; ROS_HOSTNAME: ip of the docker container (same as that of the remote computer in this case)

