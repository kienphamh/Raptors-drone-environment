# Raptors-drone-environment

A Docker development environment for the Raptors Drone Team. It bundles ROS2 (Jazzy), Gazebo (gz sim), ArduPilot (SITL), MAVROS/MAVProxy and helpful shell aliases to run, test and develop drone software in a single container.

## Features
- ROS2 Jazzy environment pre-installed
- Gazebo (gz sim harmonic) with ArduPilot Gazebo plugin
- ArduPilot (SITL) cloned and built inside the image
- MAVROS and MAVProxy installed (with GeographicLib datasets)
- Convenience aliases and helper scripts in the `aliases/` directory
- X11 forwarding support for GUI tools and Gazebo

## Repository layout
- File: [Dockerfile](Dockerfile) — container build definition
- File: [docker-compose.yml](docker-compose.yml) — compose service for `drone-sim`
- File: [setup-x11.sh](setup-x11.sh) — helper to prepare host X11 forwarding
- Directory: [aliases/](aliases/) — a set of shell helpers and shortcuts (startup, control, monitoring, navigation, development, safety, etc.)
- Directory: [workspace/src](workspace/src) — mount point for your ROS2 packages and code

## Requirements
- Docker (and docker-compose) on your host
- X server on host (Linux desktop) for GUI forwarding to work
- (Optional) GPU/device passthrough if you want hardware acceleration

## Quick start
1. Make X11 usable by the container (on your host):

```bash
./setup-x11.sh
```

2. Build the image and start the container:

```bash
docker compose build
docker compose up -d
```

3. Enter the running container shell:

```bash
docker exec -it raptors-drone-dev bash
```

Inside the container you can use the provided aliases (they are sourced from `/root/aliases` in the Dockerfile). To view the full command reference run:

```bash
help
# or
h
```

## Common commands
- `start_ardupilot` — run ArduPilot SITL with Gazebo
- `start_gazebo` — start Gazebo (gz sim)
- `start_mavros` — start the MAVROS node
- `arm` / `disarm` — arm/disarm motors via MAVROS services
- `takeoff <alt>` / `land` — basic flight commands
- `goto <lat> <lon> [alt]` — push a waypoint mission
- `record [name]` / `playback <name>` — ros2 bag recording and playback
- `build` / `rebuild` — `colcon` helpers for building ROS2 workspace

See the aliases files for the full, detailed list: [aliases/](aliases/)

## Development
- Mount your workspace into `/workspace/src` (this is already configured in `docker-compose.yml`).
- Build the workspace inside the container:

```bash
ws
build
```

Or for a rebuild that sources the workspace:

```bash
rebuild
```

## X11 / GUI notes
- `setup-x11.sh` prepares a `.docker.xauth` file and runs `xhost +local:docker` so GUI apps (Gazebo, Rviz, MAVProxy GUI) can be forwarded from the container to your host X server.
- The Dockerfile sets `DISPLAY` and `QT_X11_NO_MITSHM=1`. If forwarding fails, verify `DISPLAY` on the host and the presence of `/tmp/.X11-unix` and `/tmp/.docker.xauth` volume mounts.

## Volumes and persistence
- `docker-compose.yml` creates a named volume `arpudpilot_build` (persisting ArduPilot build artifacts) and mounts `./workspace/src` into the container so your source files are editable on the host and available inside the container.

## Safety and testing
- Use `STOP` (alias) for emergency stop — this calls `disarm` and attempts to stop motors and setpoint publishers.
- Be careful when using `disable_safety_checks`; only disable checks for controlled test environments.

## Customization
- To enable GPU/device passthrough for better Gazebo performance, uncomment and adapt the `devices:` section in `docker-compose.yml`.