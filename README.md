# Raptors-drone-environment

A Docker development environment for the Raptors Drone Team. It bundles ROS2 (Jazzy), Gazebo (gz sim), ArduPilot (SITL), MAVROS/MAVProxy and helpful shell aliases to run, test and develop drone software in a single container.

## Features
- ROS2 Jazzy environment pre-installed
- Gazebo (gz sim harmonic) with ArduPilot Gazebo plugin
- ArduPilot (SITL) cloned and built inside the image
- MAVROS and MAVProxy installed (with GeographicLib datasets)
- Convenience aliases and helper scripts in the `aliases/` directory
- X11 forwarding support for GUI tools and Gazebo

## Layout

- [docker-compose.yml](docker-compose.yml) — compose service and volume configuration (repo root)
- [.devcontainer/](.devcontainer/) — Dockerfile(s), build context, and devcontainer configuration
- [setup-x11.sh](setup-x11.sh) — prepares X11 forwarding for GUI apps
- [aliases/](aliases/) — convenience scripts and command aliases used in the container
- [workspace/src](workspace/src) — host-mounted ROS2 workspace for your packages

## Quickstart

1. Prepare X11 forwarding on the host:

```bash
sudo bash ./setup-x11.sh
```

2. Build and start the environment:
```bash
docker compose build
docker compose up -d
```

3. Enter the running container shell:

```bash
docker exec -it raptors-drone-dev bash
```

Run `help` or `h` inside the container to list provided helper aliases.

## Common commands (aliases)

- `start_ardupilot` — run ArduPilot SITL with Gazebo
- `start_gazebo` — start Gazebo (gz-sim)
- `start_mavros` — start the MAVROS node
- `arm` / `disarm` — arm/disarm via MAVROS
- `takeoff <alt>` / `land` — basic flight commands
- `goto <lat> <lon> [alt]` — send a waypoint
- `record [name]` / `playback <name>` — ros2 bag helpers
- `build` / `rebuild` — `colcon` helpers for building the ROS2 workspace

See the `.devcontainer/aliases/` directory for the full command reference.

## Customization
- To enable GPU usage for Gazebo, uncomment the last 3 lines in `docker-compose.yml`.

## Development workflow

Mount your project into `workspace/src` on the host and build inside the container:

```bash
ws
build
```

Use `rebuild` to clean, build, and source the workspace.