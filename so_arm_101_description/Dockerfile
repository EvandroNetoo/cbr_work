# SO-ARM-101 Description — ROS 2 Jazzy build and test image
FROM ros:jazzy AS base

ENV DEBIAN_FRONTEND=noninteractive

# Core ROS2 tools needed for building and validating the package
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        python3-colcon-common-extensions \
        python3-pytest \
        python3-pip \
    && pip3 install --break-system-packages --no-cache-dir trimesh numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Copy the package into the workspace
COPY . src/so_arm_101_description/

# Build
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select so_arm_101_description --symlink-install

# Default entrypoint: source workspace and run bash
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/constrain-window.sh /constrain-window.sh
RUN chmod +x /entrypoint.sh /constrain-window.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ---------- test stage ----------
FROM base AS test

RUN . /opt/ros/jazzy/setup.sh && \
    . /ws/install/setup.sh && \
    colcon test --packages-select so_arm_101_description --return-code-on-test-failure

# ---------- rviz stage (needs X11 / display) ----------
FROM base AS rviz

RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-jazzy-rviz2 \
        xdotool \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "so_arm_101_description", "display.launch.py"]

# ---------- gazebo stage ----------
FROM base AS gazebo

RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "so_arm_101_description", "sim.launch.py"]
