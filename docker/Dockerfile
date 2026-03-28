# 0. Base (Using standard jazzy tag to avoid pull errors)
FROM ros:jazzy

# 1. MIRROR swap & Noble Updates Fix
RUN sed -i \
    -e 's|http://gb.archive.ubuntu.com/ubuntu/|https://mirror.accum.se/ubuntu/|g' \
    -e 's|http://archive.ubuntu.com/ubuntu/|https://mirror.accum.se/ubuntu/|g' \
    /etc/apt/sources.list.d/ubuntu.sources || true && \
    sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-security/' \
    /etc/apt/sources.list.d/ubuntu.sources





# 2. Locale & Timezone
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 3. Hardened System Dependencies (No libopencv-dev)
RUN apt-get update && \
    apt-get install -y -o Acquire::Retries=10 --fix-missing --no-install-recommends \
    wget ca-certificates unzip git python3-pip python3-rosdep curl gnupg2 \
    libusb-1.0-0-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 4. L-CAS Repository
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://lcas.lincoln.ac.uk/apt/repo_signing.key | \
    gpg --dearmor -o /usr/share/keyrings/lcas-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/lcas-keyring.gpg] https://lcas.lincoln.ac.uk/apt/lcas noble lcas" > /etc/apt/sources.list.d/lcas.list
    
# 4b. Mirror ROS2 packages repo to avoid 400 Bad Request from CDN
RUN sed -i 's|http://packages.ros.org/ros2/ubuntu|https://packages.ros.org/ros2/ubuntu|g' \
    /etc/apt/sources.list.d/ros2-latest.list 2>/dev/null || \
    sed -i 's|http://packages.ros.org/ros2/ubuntu|https://packages.ros.org/ros2/ubuntu|g' \
    /etc/apt/sources.list.d/*.list 2>/dev/null || true && \
    echo 'Acquire::Retries "10";' > /etc/apt/apt.conf.d/80retries && \
    echo 'Acquire::http::Timeout "120";' >> /etc/apt/apt.conf.d/80retries && \
    echo 'Acquire::https::Timeout "120";' >> /etc/apt/apt.conf.d/80retries
    
# 5. Core tools & Manual Nav2/Control Fix (Forces refresh to fix 400 Bad Request)
ARG CACHE_BUST=1
RUN for i in 1 2 3 4 5; do \
      apt-get update && \
      apt-get install -y --fix-missing --no-install-recommends \
        ros-jazzy-septentrio-gnss-driver \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-rtcm-msgs \
        ros-jazzy-usb-cam \
        ros-jazzy-image-transport \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-tf-transformations \
        ros-jazzy-interactive-markers \
        ros-jazzy-nav2-behavior-tree \
        ros-jazzy-nav2-bt-navigator \
        ros-jazzy-control-msgs \
        ros-jazzy-controller-manager-msgs \
        python3-serial python3-requests python3-yaml \
      && break || (echo "Attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
    
# 6. LIZARD HARDENING
RUN mkdir -p /root/.lizard && cd /root && \
    LIZARD_VERSION=v0.9.0 && \
    wget --tries=10 --waitretry=5 --retry-connrefused \
    https://github.com/zauberzeug/lizard/releases/download/${LIZARD_VERSION}/lizard_firmware_and_devtools_${LIZARD_VERSION}_esp32.zip && \
    unzip -qo *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 -r /root/.lizard/requirements.txt

# 7. PIP HARDENING (Includes Headless OpenCV)
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed --retries 10 \
    --timeout 1000 --default-timeout=1000 \
    opencv-python-headless \
    -r /tmp/requirements.txt

# 8. ENVIRONMENT
COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc
ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive

# 9. WORKSPACE SETUP
WORKDIR /workspace
COPY ./devkit_driver ./src/devkit_driver
COPY ./devkit_launch ./src/devkit_launch
COPY ./devkit_ui ./src/devkit_ui

RUN GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b main https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss && \
    GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b aoc_refactor https://github.com/LCAS/topological_navigation src/topological_navigation && \
    GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b feat/gazebo-harmonic-port https://github.com/samuk/caatingarobotics src/caatingarobotics && \
    GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b master https://github.com/LCAS/sentor src/sentor

# 9.1: Automated Dependency Resolution
RUN apt-get update && apt-get install -y --fix-broken && \
    if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    for i in 1 2 3 4 5; do \
      apt-get update && \
      rosdep install --from-paths src --ignore-src -y \
      --rosdistro jazzy \
      --skip-keys "\
      libopencv-dev \
      opencv-python-headless \
      PyQt5 \
      PyQtWebEngine \
      ros-jazzy-cv-bridge \
      ros-jazzy-compressed-image-transport \
      ros-jazzy-compressed-depth-image-transport \
      ros-jazzy-theora-image-transport \
      ros-jazzy-nav2-behavior-tree \
      ros-jazzy-nav2-bt-navigator \
      ros-jazzy-tf-transformations \
      tf_transformations \
      interactive_markers \
      topological_navigation \
      topological_navigation_msgs \
      topological_navigation_visual \
      topological_nav_simulator \
      mongodb_store \
      strands_navigation_msgs \
      nicegui \
      septentrio_gnss_driver \
      ublox_dgnss \
      rtcm_msgs \
      transforms3d \
      pyserial \
      usb_cam \
      image_transport \
      foxglove_bridge \
      lizard \
      ament_python \
      gazebo_ros \
      agro_robot_sim \
      caatinga_nav \
      caatinga_vision \
      sowbot_row_follow \
      ultralytics \
      sentor" \
      && break || (echo "Attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    rm -rf /var/lib/apt/lists/*

# 10. Manual PIP Overrides (No PyQt)
RUN pip install --break-system-packages --no-cache-dir \
    nicegui pyserial transforms3d lizard pymongo networkx \
    ultralytics 

# 11. BUILD
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 12. ENTRYPOINT
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
