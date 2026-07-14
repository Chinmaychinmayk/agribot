# Autonomous Farm Lane Navigation - ROS 2 Humble + Gazebo Classic + Nav2 + RTAB-Map
#
# Humble targets Ubuntu 22.04, so we build on the official ROS image rather than
# the host's Ubuntu 24.04 (where Humble has no native packages).
#
# Build:  docker build -t farm_nav .
# Run  :  ./run.sh            (see that script / README for X11 forwarding)

FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ---- ROS / sim dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-image-transport \
    python3-opencv \
    python3-matplotlib \
    python3-numpy \
    python3-colcon-common-extensions \
    ffmpeg \
    xvfb \
    x11-utils \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# headless software GL for Gazebo camera sensors without a GPU
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

# ---- build the workspace ----
WORKDIR /farm_nav_ws
COPY src ./src
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install

# auto-source for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /farm_nav_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
