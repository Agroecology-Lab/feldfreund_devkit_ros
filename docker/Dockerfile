FROM ros:jazzy

# 0. MIRROR swap (UK Specific)
RUN sed -i 's|http://gb.archive.ubuntu.com/ubuntu/|http://mirror.ox.ac.uk/ubuntu/|g' /etc/apt/sources.list.d/ubuntu.sources || true

# 1. Core tools & Essential ROS Binaries
# We combine these to reduce image layers and avoid the double-repo conflict
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget ca-certificates unzip git python3-pip python3-rosdep \
    libusb-1.0-0-dev libopencv-dev \
    ros-jazzy-septentrio-gnss-driver \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rtcm-msgs \
    ros-jazzy-usb-cam \
    ros-jazzy-image-transport \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-tf2-geometry-msgs \
    python3-serial python3-requests python3-yaml \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 2. LIZARD HARDENING
RUN mkdir -p /root/.lizard && cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    wget --tries=10 --waitretry=5 --retry-connrefused \
    https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && \
    unzip -qo *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 -r /root/.lizard/requirements.txt

# 3. PIP HARDENING
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed --retries 10 \
    --timeout 1000 --default-timeout=1000 -r /tmp/requirements.txt

# 4. ENVIRONMENT
COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc
ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages:${PYTHONPATH:+:${PYTHONPATH}}"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive

# 5. WORKSPACE SETUP
WORKDIR /workspace
COPY ./devkit_driver ./src/devkit_driver
COPY ./devkit_launch ./src/devkit_launch
COPY ./devkit_ui ./src/devkit_ui

RUN GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b main https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss

# 5.1: Automated Dependency Resolution (Clean Syntax)
RUN if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --rosdistro jazzy \
    --skip-keys "\
    nicegui \
    lizard \
    transforms3d \
    pyserial \
    septentrio_gnss_driver \
    ublox_dgnss \
    rtcm_msgs \
    usb_cam \
    image_transport \
    foxglove_bridge \
    tf-transformations \
    ros-jazzy-tf-transformations \
    tf_transformations \
    ros-jazzy-tf-transformations" && \
    rm -rf /var/lib/apt/lists/*

# 6. Manual PIP Overrides
RUN pip install --break-system-packages --no-cache-dir \
    nicegui pyserial transforms3d lizard pymongo networkx

# 7. BUILD
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 8. ENTRYPOINT
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
