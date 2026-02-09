#!/bin/bash

get_param() {
    if [ -z "$1" ]; then
        echo "Usage: get_param <param_name>"
        echo "Example: get_param WPNAV_SPEED"
        return 1
    fi
    ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet "{param_id: '$1'}"
}

set_param() {
    if [ -z "$1" ] || [ -z "$2" ]; then
        echo "Usage: set_param <param_name> <value>"
        echo "Example: set_param WPNAV_SPEED 500"
        return 1
    fi
    ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet \
        "{param_id: '$1', value: {integer: $2, real: 0.0}}"
}

save_params() {
    echo "Saving parameters to EEPROM..."
    ros2 service call /mavros/param/push mavros_msgs/srv/ParamPush
    echo "Parameters saved"
}