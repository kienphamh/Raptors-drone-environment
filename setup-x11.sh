#!/bin/bash
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker
echo "X11 forwarding enabled for Docker"
echo "You can now run: docker compose up"