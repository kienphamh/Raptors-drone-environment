#!/bin/bash
XAUTH=/tmp/.docker.xauth
echo "Setting up X11 forwarding for Docker..."
touch $XAUTH
chmod 666 $XAUTH
if [ -n "$DISPLAY" ]; then
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null
else
    echo " Warning: DISPLAY variable not set"
    echo " X11 forwarding may not work"
fi
xhost +local:docker 
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "X11 forwarding has been enabled!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps:"
echo "  1. docker compose build"
echo "  2. docker compose up -d"
echo "  3. docker exec -it raptors-drone-dev bash"
echo ""