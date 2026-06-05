#!/bin/bash
EMERGENCY_STOP() {
    echo "EMERGENCY STOP - KILLING MOTORS"
    disarm
    pkill -f "setpoint"
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
}

alias STOP='EMERGENCY_STOP'

disable_safety_checks() {
    echo "WARNING: Disabling pre-arm and safety checks..."
    set_param ARMING_CHECK 0
    set_param DISARM_DELAY 0
    echo "Safety checks disabled!"
}

reenable_safety_checks() {
    echo "Enabling safety checks..."
    set_param ARMING_CHECK 1
    set_param DISARM_DELAY 10
    echo "Safety checks re-enabled!"
}

set_geofence() {
    local radius=${1:-100}
    echo "Setting geofence radius to ${radius}m"
    set_param FENCE_RADIUS $radius
    set_param FENCE_ENABLE 1
    echo "Geofence enabled"
}