#!/bin/bash

echo "Enter this device's IP address (pi): (without http:// or port)"
read IP
export ROS_MASTER_URI=http://$IP:11311
export ROS_IP=$IP
unset ROS_HOSTNAME # Setting both ROS_IP and ROS_HOSTNAME isn't necessary and it might cause problems if they're different

source /opt/ros/noetic/setup.bash

if [ -f "devel/setup.bash" ]; then
    # echo "Sourcing devel/setup.bash"
    source devel/setup.bash
elif [ -f "devel_isolated/setup.bash" ]; then
    # echo "Sourcing devel_isolated/setup.bash"
    source devel_isolated/setup.bash
else
    echo "Could not find a ROS setup.bash file! ROS commands may not work."
fi

echo "Done."