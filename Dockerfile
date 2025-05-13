FROM osrf/ros:humble-desktop

RUN apt-get update \
    && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /ros2_ws
COPY src    src
COPY models models

RUN apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash","-lc","ros2 launch warde_bt launch_all.launch.py"]
