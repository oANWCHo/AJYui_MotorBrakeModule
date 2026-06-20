# ROS 2 Jazzy (Ubuntu 24.04, Python 3.12) container for the MotorBrake CAN bridge.
# Lets you run the bridge on hosts where ROS 2 isn't / can't be installed natively
# (e.g. Ubuntu 26.04 + Python 3.14). SocketCAN is shared from the host via
# `--network host`, so `can0` must be brought up on the HOST, not in here.
FROM ros:jazzy

# python-can (CAN backend) + can-utils (candump/cansend for debugging) + colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-can \
        can-utils \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Build the two packages into an overlay workspace.
WORKDIR /ros2_ws
COPY motorbrake_msgs   src/motorbrake_msgs
COPY motorbrake_bridge src/motorbrake_bridge
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_direct+

# Auto-source for interactive shells (docker exec ... bash).
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "motorbrake_bridge", "brake_bridge"]
