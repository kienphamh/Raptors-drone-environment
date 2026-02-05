#!/bin/bash

# Start
start_ardupilot() {
    cd $ARDUPILOT_HOME
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
}

start_gazebo() {
    gz sim -v4 -r $GAZEBO_WORLD
}

start_mavros() {
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=$FCU_URL \
        -p target_system_id:=1 \
        -p target_component_id:=1
}

start_all() {
    echo "Starting the whole drone stack..."
    
    echo "Starting Gazebo..."
    gz sim -v4 -r $GAZEBO_WORLD &
    sleep 5
    
    echo "Starting ArduPilot SITL..."
    cd $ARDUPILOT_HOME
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console &
    sleep 10
    
    echo "Starting MAVROS..."
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=$FCU_URL \
        -p target_system_id:=1 \
        -p target_component_id:=1 &
    
    echo "All components started!"
    echo "Use 'arm' and 'takeoff <altitude>' to fly"
}

# Stop
stop_gazebo() {
    pkill -f "gz sim"
    echo "Gazebo stopped"
}

stop_ardupilot() {
    pkill -f "sim_vehicle"
    echo "ArduPilot stopped"
}

stop_mavros() {
    pkill -f "mavros_node"
    echo "MAVROS stopped"
}

stop_all() {
    echo "Stopping all drone processes..."
    pkill -f "gz sim"
    pkill -f "sim_vehicle"
    pkill -f "mavros_node"
    echo "All processes stopped"
}

# Start only MAVROS (for hardware testing)
start_hardware() {
    echo "Starting MAVROS for hardware connection..."
    echo "Make sure your flight controller is connected!"
    
    # For hardware, typically use serial connection
    local hw_fcu_url=${1:-/dev/ttyACM0:57600}
    
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=$hw_fcu_url \
        -p target_system_id:=1 \
        -p target_component_id:=1
}

# Minimal start (MAVROS only, for testing)
start_minimal() {
    echo "Starting minimal setup (MAVROS only)..."
    start_mavros
}

# Restart specific component
restart_mavros() {
    echo "Restarting MAVROS..."
    stop_mavros
    sleep 2
    start_mavros
}

restart_gazebo() {
    echo "Restarting Gazebo..."
    stop_gazebo
    sleep 2
    start_gazebo
}

restart_ardupilot() {
    echo "Restarting ArduPilot..."
    stop_ardupilot
    sleep 2
    start_ardupilot
}