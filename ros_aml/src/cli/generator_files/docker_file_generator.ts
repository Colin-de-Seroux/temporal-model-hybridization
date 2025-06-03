export function generateDockerfile(
    pkgName: string,
    rosVersion: string = 'jazzy',
    wantLogging: boolean = true
): string {
    return `
FROM ros:${rosVersion}

# Supprimer toutes les références ROS 2 dans les sources (clé expirée)
RUN rm -f /etc/apt/sources.list.d/ros2* \
    && sed -i '/packages.ros.org/d' /etc/apt/sources.list
RUN apt-get update && apt-get install -y curl gnupg


RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list


RUN apt-get update && apt-get install -y \\
    python3-colcon-common-extensions \\
    python3-pip \\
    build-essential && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Create logs dir
${wantLogging ? 'RUN mkdir -p /ros2_ws/.ros/log' : ''}

# Copy ${pkgName} package
COPY . /ros2_ws/src/${pkgName}

# Install dependencies
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install

# New entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
`.trim();
}

export function generateDockerComposePart(pkgName: string, wantLogging: boolean = true): string {
    return `
services:
  ${pkgName}:
    image: ${pkgName}
    build:
      context: ./ros2/src/${pkgName}
      dockerfile: Dockerfile
    container_name: ${pkgName}
${wantLogging ? `    environment:
      - RCUTILS_LOGGING_USE_STDERR=0 # Logs STDERR
      - RCUTILS_LOGGING_USE_STDOUT=0 # Logs STDOUT
      - RCUTILS_LOGGING_IMPLEMENTATION=rcutils_logging_file
      - ROS_HOME=/ros2_ws/.ros
    volumes:
      - ./ros2/src/${pkgName}/Logs:/ros_logs_backup
    tmpfs:
      - /ros2_ws/.ros/log # To RAM` : ''}
    deploy:
      resources:
        limits:
          cpus: "1"
          memory: 200M
        reservations:
          cpus: "1"
          memory: 200M
`.trim();
}
