# ==========================================
# STAGE 1: env-base (Shared dependencies)
# ==========================================
FROM ros:jazzy-ros-base AS env-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
# Set CycloneDDS as the default RMW for dual-interface reliability
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SHELL ["/bin/bash", "-c"]

### SYSTEM UTILS (Crucial for later steps) ###
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    unzip \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

### SYSTEM & ROS DEPENDENCIES ###
RUN echo "Acquire::http::Pipeline-Depth 0;" > /etc/apt/apt.conf.d/99nopipeline && \
    apt-get update && \
    for i in {1..5}; do \
    apt-get install -y --no-install-recommends \
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
    python3-pip \
    python3-serial \
    python3-requests \
    python3-yaml \
    nano \
    iputils-ping \
    && break || (echo "Apt failed, retrying in 10s..." && sleep 10); \
    done && \
    rm -rf /var/lib/apt/lists/*

### LIZARD SETUP ###
RUN mkdir -p /root/.lizard && \
    cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    for i in {1..5}; do \
    wget -q https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && break || \
    (echo "Lizard download failed, retrying..." && sleep 5); \
    done && \
    unzip -qo lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip -d /root/.lizard && \
    chmod +x /root/.lizard/espresso.py && \
    cd /root/.lizard && \
    pip install --break-system-packages --ignore-installed -r requirements.txt && \
    rm /root/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip

### PYTHON PIP DEPENDENCIES ###
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    for i in {1..5}; do \
    pip install --break-system-packages --ignore-installed -r /tmp/requirements.txt && break || \
    (echo "Pip failed, retrying..." && sleep 5); \
    done

# ==========================================
# STAGE 2: builder (Compiler & Source)
# ==========================================
FROM env-base AS builder
# Install build-only tools
RUN apt-get update && apt-get install -y python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY ./devkit_driver /workspace/src/devkit_driver
COPY ./devkit_launch /workspace/src/devkit_launch
COPY ./devkit_ui /workspace/src/devkit_ui

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==========================================
# STAGE 3: runtime (Slim final image)
# ==========================================
FROM env-base AS runtime
WORKDIR /workspace

# Copy build artifacts from stage 2
COPY --from=builder /workspace/install /workspace/install
# Optional: Copy src if you need Python node source access at runtime
COPY --from=builder /workspace/src /workspace/src

# Apply bashrc customizations
COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

# Set Paths for Lizard and built ROS packages
ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

# Healthcheck/Environment validation
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash; exec \"$@\"", "--"]
CMD ["bash"]
