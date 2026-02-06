#!/bin/bash
# Core environment variables and configuration

export ARDUPILOT_HOME=${ARDUPILOT_HOME:-/opt/ardupilot/ArduCopter}
export FCU_URL=${FCU_URL:-udp://:14550@127.0.0.1:14550@}

# Workspace shortcuts
alias ws='cd /workspace'
alias src='source /workspace/install/setup.bash'