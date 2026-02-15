# BUILD: docker build -t feldfreund:jazzy .
FROM ros:jazzy
RUN echo "Acquire::http::Timeout \"300\";\nAcquire::ftp::Timeout \"300\";\nAcquire::Retries \"20\";" > /etc/apt/apt.conf.d/99timeout

# 0. RESILIENCE: Mirror Swap (Optional: Remove if you still get 400 Bad Request)
#RUN sed -i 's|http://gb.archive.ubuntu.com/ubuntu/|https://mirror.accum.se/ubuntu/|g' /etc/apt/sources.list.d/ubuntu.sources || true
RUN sed -i 's|http://.*archive.ubuntu.com/ubuntu/|https://mirror.init7.net/ubuntu/|g' /etc/apt/sources.list.d/ubuntu.sources || true

# 1. LOCALES & TIMEZONE
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 DEBIAN_FRONTEND=noninteractive

# 2. CORE UTILITIES & PYTHON (Non-negotiable tools)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    wget curl unzip ca-certificates gnupg2 python3-pip python3-setuptools python3-rosdep && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. L-CAS REPOSITORY
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://lcas.lincoln.ac.uk/apt/repo_signing.key | \
    gpg --dearmor -o /usr/share/keyrings/lcas-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/lcas-keyring.gpg] https://lcas.lincoln.ac.uk/apt/lcas noble lcas" > /etc/apt/sources.list.d/lcas.list


# Manually fetch with SSL check disabled to bypass exit code 5
RUN wget --no-check-certificate --timeout=30 --tries=10 -O /tmp/iceoryx.deb \
    "https://packages.ros.org/ros2/ubuntu/pool/main/r/ros-jazzy-iceoryx-binding-c/ros-jazzy-iceoryx-binding-c_2.0.6-1noble.20260121.180617_amd64.deb" && \
    (dpkg -i /tmp/iceoryx.deb || apt-get install -y -f) && \
    rm -f /tmp/iceoryx.deb



# 4. SYSTEM & ROS BINARIES
# We explicitly include rtcm-msgs and gazebo-ros here to satisfy CMake
RUN apt-get update && \
    apt-get install -y --fix-missing -o Acquire::Retries=10 --no-install-recommends \
    git libusb-1.0-0-dev libopencv-dev iputils-ping nano \
    libcurl4t64 libcurl4-openssl-dev || true && \
    apt-get install -y --fix-missing -o Acquire::Retries=10 --no-install-recommends \
    ros-jazzy-septentrio-gnss-driver \
    ros-jazzy-iceoryx-hoofs \
    ros-jazzy-iceoryx-posh \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rtcm-msgs \
    ros-jazzy-usb-cam \
    ros-jazzy-image-transport \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-foxglove-msgs \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz \
    ros-jazzy-tf-transformations \
    ros-jazzy-cv-bridge \
    python3-serial python3-requests python3-yaml \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 5. LIZARD INSTALL
RUN mkdir -p /root/.lizard && cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    wget --tries=10 --waitretry=5 --retry-connrefused \
    https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && \
    unzip -qo *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 -r /root/.lizard/requirements.txt

# 6. PIP REQUIREMENTS
COPY requirements.txt /tmp/requirements.txt

RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed \
    --retries 10 --timeout 120 \
    nicegui transforms3d lizard pymongo networkx pyserial scipy -r /tmp/requirements.txt

# 7. WORKSPACE SETUP
WORKDIR /workspace
COPY ./devkit_driver ./src/devkit_driver
COPY ./devkit_launch ./src/devkit_launch
COPY ./devkit_ui ./src/devkit_ui

RUN GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b main https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss && \
    GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b jazzy https://github.com/samuk/caatingarobotics src/caatingarobotics

# 8. ROSDEP & FULL BUILD
RUN if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    --skip-keys "nicegui lizard transforms3d pyserial septentrio_gnss_driver rtcm_msgs gazebo_ros gazebo_ros_pkgs ament_python gps_msgs libusb-1.0-dev tf_transformations xacro image_transport usb_cam libcurl4-openssl-dev libcurl4 libcurl4t64 libcurl-dev libcurl curl diagnostic_updater diagnostic_msgs joint_state_publisher joint-state-publisher vision_msgs cv_bridge cv-bridge" && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 9. RUNTIME
COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
