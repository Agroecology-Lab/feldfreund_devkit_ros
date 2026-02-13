FROM ros:jazzy

# 0. MIRROR swap (Cloudflare Resilience)
# Cloudflare's mirror is often faster and less prone to the 408 Timeouts you're seeing
RUN sed -i 's|http://gb.archive.ubuntu.com/ubuntu/|https://mirror.accum.se/ubuntu/|g' /etc/apt/sources.list.d/ubuntu.sources || true && \
    sed -i 's|http://archive.ubuntu.com/ubuntu/|https://mirror.accum.se/ubuntu/|g' /etc/apt/sources.list.d/ubuntu.sources || true

# 1. Locale & Timezone
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 2. Hardened System Dependencies (With Retries)
# We set Acquire::Retries to 10 to push through network blips
RUN apt-get update && \
    apt-get install -y -o Acquire::Retries=10 --fix-missing --no-install-recommends \
    wget ca-certificates unzip git python3-pip python3-rosdep curl gnupg2 \
    libusb-1.0-0-dev libopencv-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. L-CAS Repository (Noble Only)
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://lcas.lincoln.ac.uk/apt/repo_signing.key | \
    gpg --dearmor -o /usr/share/keyrings/lcas-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/lcas-keyring.gpg] https://lcas.lincoln.ac.uk/apt/lcas noble lcas" > /etc/apt/sources.list.d/lcas.list

# 4. ROS Jazzy Binaries
RUN apt-get update && \
    apt-get install -y -o Acquire::Retries=10 --no-install-recommends \
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

# 1. Core tools & Essential ROS Binaries
# Added --fix-missing and an automatic retry for apt
RUN apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
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
# Specifically targets the ESP32 devtools release for hardware orchestration
RUN mkdir -p /root/.lizard && cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    wget --tries=10 --waitretry=5 --retry-connrefused \
    https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && \
    unzip -qo *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 -r /root/.lizard/requirements.txt

# 3. PIP HARDENING
# Uses BuildKit cache mount to accelerate repetitive builds
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed --retries 10 \
    --timeout 1000 --default-timeout=1000 -r /tmp/requirements.txt

# --- Section 4 (Environment) ---
# This removes the "UndefinedVar" warning by setting a clean initial path
ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive

# 5. WORKSPACE SETUP
WORKDIR /workspace

# 5.1: Copy existing host source 
COPY ./src ./src

# 5.2: Hardware-specific dependencies from source
# 1. Added 'src/' to the caatingarobotics clone path
# 2. Added diagnostics (almost certain to be the next error if skipped)
RUN GIT_TERMINAL_PROMPT=0 \
    git clone -b ros2 https://github.com/ros-perception/vision_msgs.git src/vision_msgs && \
    git clone -b ros2 https://github.com/ros/joint_state_publisher.git src/joint_state_publisher && \
    git clone -b ros2 https://github.com/ros/xacro.git src/xacro && \
    git clone -b main https://github.com/joaodemouragy-hash/caatingarobotics.git src/caatingarobotics && \
    if [ ! -d "src/ublox_dgnss" ]; then \
        git clone --depth 1 -b main https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss; \
    fi
# ---  Section 5.3 (Rosdep) ---
RUN if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --rosdistro jazzy \
    --skip-keys "\
    python_qt_binding \
    diagnostic_updater \
    diagnostic_msgs \
    rviz_common \
    rviz_rendering \
    rviz_default_plugins \
    rviz_ogre_vendor \
    rviz2 \
    joint_state_publisher \
    xacro \
    vision_msgs \
    ament_python \
    ament_cmake_python \
    gazebo_ros \
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
    tf_transformations" && \
    rm -rf /var/lib/apt/lists/*

# 6. Manual PIP Overrides
# Ensures specific high-level libraries are available in the system-site-packages
RUN pip install --break-system-packages --no-cache-dir \
    nicegui pyserial transforms3d lizard pymongo networkx
   
   
# 7. BUILD
# We use --packages-skip to ignore all GUI and simulation related packages
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install \
    --packages-skip \
    agro_robot_sim \
    joint_state_publisher_gui \
    vision_msgs_rviz_plugins \
    rviz_common \
    rviz_rendering \
    --cmake-args -DCMAKE_BUILD_TYPE=Release   

# 8. ENTRYPOINT
# Ensures the ROS environment is sourced for every command, with exec to handle signals properly
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
