FROM ros:latest

RUN apt-get update && apt-get install -y \
      python3-pip \
      python3-colcon-common-extensions \
      python3-setuptools \
      python3-setuptools-scm \
      python3-build \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

COPY ./src/timer_execution ./timer_execution

WORKDIR /ros2_ws
#RUN . /opt/ros/humble/setup.sh && colcon build
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

#CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener_launch.py"]
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run timer_execution my_node"]
