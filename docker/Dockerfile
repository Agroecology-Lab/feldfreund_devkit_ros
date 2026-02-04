FROM ros:jazzy

# 1. APT HARDENING: Core utilities + ROS packages in one cached layer
# Using cache mounts prevents re-downloading 1GB of data if a later step fails.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    wget ca-certificates unzip git nano iputils-ping python3-pip \
    ros-jazzy-domain-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-xacro \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-septentrio-gnss-driver \
    ros-jazzy-usb-cam \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-image-tools \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-axis-camera \
    python3-serial python3-requests python3-yaml && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 2. LIZARD HARDENING: Survives GitHub timeouts
RUN mkdir -p /root/.lizard && cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    wget --tries=10 --waitretry=5 --retry-connrefused \
    https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && \
    unzip -qo *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 -r /root/.lizard/requirements.txt

# 3. PIP HARDENING: General requirements
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed --retries 10 -r /tmp/requirements.txt

# 4. ENVIRONMENT & PERSISTENCE
COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

# Set critical ENVs so they are visible to non-interactive shells (like manage.py)
ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages:${PYTHONPATH:+:${PYTHONPATH}}"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive

# 5. WORKSPACE BUILD: Static build (No symlinks to break)
WORKDIR /workspace
COPY ./devkit_driver ./src/devkit_driver
COPY ./devkit_launch ./src/devkit_launch
COPY ./devkit_ui ./src/devkit_ui

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select devkit_driver devkit_launch devkit_ui && \
    rm -rf build/ log/

# 6. SMOKE TEST: Verification
RUN ls /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so || (echo "FATAL: CycloneDDS missing!" && exit 1)
RUN ls /workspace/install/devkit_launch/share/devkit_launch/launch/devkit.launch.py || (echo "FATAL: Launch files not found!" && exit 1)

# 7. ENTRYPOINT: The "Manage.py" Insurance Policy
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
