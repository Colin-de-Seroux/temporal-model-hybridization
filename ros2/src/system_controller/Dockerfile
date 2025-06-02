FROM ros:jazzy

# Supprimer toutes les références ROS 2 dans les sources (clé expirée)
RUN rm -f /etc/apt/sources.list.d/ros2*     && sed -i '/packages.ros.org/d' /etc/apt/sources.list
RUN apt-get update && apt-get install -y curl gnupg


RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg &&     echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list


RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Create logs dir
RUN mkdir -p /ros2_ws/.ros/log

# Copy system_controller package
COPY . /ros2_ws/src/system_controller

# Install dependencies
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install

# New entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]