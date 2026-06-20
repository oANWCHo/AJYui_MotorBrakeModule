#!/usr/bin/env bash
# Run the MotorBrake ROS 2 <-> CAN bridge container.
#
#   ./run.sh                 -> start the bridge node
#   ./run.sh bash            -> open a shell inside (ROS already sourced)
#   ./run.sh ros2 topic echo /brake_status
#
# Requires can0 to be UP on the host first:
#   sudo ip link set can0 up type can bitrate 1000000
set -euo pipefail

IMAGE="${IMAGE:-motorbrake-bridge}"

# --network host  : share the host's SocketCAN (can0) and DDS discovery
# --ipc=host      : share /dev/shm so Fast DDS shared-memory transport works
#                   ACROSS containers (without this, topics discover but data
#                   never flows between separate containers)
# --cap-add NET_ADMIN : allow `ip link` inside the container if ever needed
exec docker run --rm -it \
    --network host \
    --ipc=host \
    --cap-add NET_ADMIN \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "$IMAGE" "$@"
