#!/bin/bash

# Source ROS2 environment
if [[ -f /opt/ros/$ROS_DISTRO/setup.bash ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Source ROS2 aliases
if [[ -f /home/dev/ros2-aliases/ros2_aliases.bash ]]; then
    source /home/dev/ros2-aliases/ros2_aliases.bash
fi

# Source workspace if it exists
if [[ -f /home/dev/ros2_ws/install/setup.bash ]]; then
    source /home/dev/ros2_ws/install/setup.bash
fi

# Execute the provided command
exec "$@"
