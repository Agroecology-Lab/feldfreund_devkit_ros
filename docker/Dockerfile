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

# 4. ROS Jazzy System & Build Dependencies
RUN apt-get update && \
    apt-get install -y -o Acquire::Retries=10 --fix-missing --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    qt6-base-dev \
    libqt6svg6-dev \
    libogre-1.12-dev \
    libusb-1.0-0-dev \
    libcurl4-openssl-dev \
    python3-serial \
    python3-requests \
    python3-yaml \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 5. LIZARD HARDENING
# Specifically targets the ESP32 devtools release for hardware orchestration
RUN mkdir -p /root/.lizard && cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    wget --tries=10 --waitretry=5 --retry-connrefused \
    https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && \
    unzip -qo *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 -r /root/.lizard/requirements.txt

# 6. PIP HARDENING
# Uses BuildKit cache mount to accelerate repetitive builds
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed --retries 10 \
    --timeout 1000 --default-timeout=1000 -r /tmp/requirements.txt

# 7. Environment
# This removes the "UndefinedVar" warning by setting a clean initial path
ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive

# 8. WORKSPACE SETUP
WORKDIR /workspace

# 9: Copy existing host source 
COPY ./src ./src

# 10: Hardware & Infrastructure dependencies from source
RUN git config --global core.askPass /bin/echo && \
    export GIT_TERMINAL_PROMPT=0 && \
    git clone --depth 1 -b ros2 https://github.com/ros-perception/vision_msgs.git src/vision_msgs && \
    git clone --depth 1 -b ros2 https://github.com/ros/joint_state_publisher.git src/joint_state_publisher && \
    git clone --depth 1 -b ros2 https://github.com/ros/xacro.git src/xacro && \
    git clone --depth 1 -b ros2 https://github.com/ros/diagnostics.git src/diagnostics && \
    git clone --depth 1 -b main https://github.com/ros-visualization/python_qt_binding.git src/python_qt_binding && \
    git clone --depth 1 -b main https://github.com/joaodemouragy-hash/caatingarobotics.git src/caatingarobotics && \
    git clone --depth 1 -b main https://github.com/ros-drivers/transport_drivers.git src/transport_drivers && \
    git clone --depth 1 -b main https://github.com/ros-drivers/usb_cam.git src/usb_cam && \
        git clone --depth 1  https://github.com/ros2/rviz src/rviz && \
    git clone --depth 1 https://github.com/ros-perception/image_transport_plugins.git src/image_transport_plugins && \
    git clone --depth 1 https://github.com/septentrio-gnss/septentrio_gnss_driver.git src/septentrio_gnss_driver && \
    git clone --depth 1 https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss && \
    git clone --depth 1 https://github.com/foxglove/ros-foxglove-bridge.git src/foxglove_bridge && \
    git clone --depth 1 -b ros2 https://github.com/ros2/geometry2.git src/geometry2
#issues with this, fix later   git clone --depth 1 https://github.com/ros-agriculture/ntrip_client_ros.git src/ntrip_client && \

# 11. (Rosdep)
RUN if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y \
    --rosdistro jazzy \
    --skip-keys "\
    catkin \
    google-benchmark \
    cmake_modules \
    message_generation \
    message_runtime \
    roscpp \
    roslib \
    rospy \
    rosconsole \
    rosgraph_msgs \
    geometry_msgs \
    resource_retriever \
    ament_python \
    gazebo_ros \
    nicegui \
    lizard \
    transforms3d \
    pyserial \
    rosunit \
    media_export \
    tf-transformations \
    tf_transformations" && \
    rm -rf /var/lib/apt/lists/*


# 12. Manual PIP Overrides
# Ensures specific high-level libraries are available in the system-site-packages
RUN pip install --break-system-packages --no-cache-dir \
    nicegui pyserial transforms3d lizard pymongo networkx
   
   
# 13. BUILD
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install \
    --packages-skip \
    agro_robot_sim \
    joint_state_publisher_gui \
    vision_msgs \
    rviz_plugins \
    rviz_common \
    rviz_rendering \
    --cmake-args -DCMAKE_BUILD_TYPE=Release  

# 14. ENTRYPOINT
# Ensures the ROS environment is sourced for every command, with exec to handle signals properly
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
