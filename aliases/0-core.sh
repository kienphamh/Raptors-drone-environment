#!/bin/bash
# Core environment variables and configuration

export ARDUPILOT_HOME=${ARDUPILOT_HOME:-/opt/ardupilot/ArduCopter}
export GAZEBO_WORLD=${GAZEBO_WORLD:-~/ardupilot_gazebo/iris_runway.sdf}
export FCU_URL=${FCU_URL:-udp://:14550@127.0.0.1:14557}

# Workspace shortcuts
alias ws='cd /workspace'
alias src='source /workspace/install/setup.bash'