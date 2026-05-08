# ============================================================
# Stage 0: Base
# ============================================================
FROM ros:jazzy

# ============================================================
# Stage 1: Mirror + Locale + Timezone
# ============================================================
RUN sed -i \
    -e 's|http://gb.archive.ubuntu.com/ubuntu/|g' \
    -e 's|http://archive.ubuntu.com/ubuntu/|g' \
    /etc/apt/sources.list.d/ubuntu.sources || true && \
    sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-security/' \
    /etc/apt/sources.list.d/ubuntu.sources || true

RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# ============================================================
# Stage 2: Third-party APT Repositories
# ============================================================

# 2.1 OSRF Gazebo repo
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# 2.2 L-CAS repo
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://lcas.lincoln.ac.uk/apt/repo_signing.key | \
    gpg --dearmor -o /usr/share/keyrings/lcas-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/lcas-keyring.gpg] \
      https://lcas.lincoln.ac.uk/apt/lcas noble lcas" \
      > /etc/apt/sources.list.d/lcas.list

# 2.3 Force HTTPS for ROS2 packages repo + apt retry hardening
RUN find /etc/apt/sources.list.d/ -name "*.list" -exec \
      sed -i 's|http://packages.ros.org/ros2/ubuntu|https://packages.ros.org/ros2/ubuntu|g' {} \; 2>/dev/null || true && \
    echo 'Acquire::Retries "10";'         >  /etc/apt/apt.conf.d/80retries && \
    echo 'Acquire::http::Timeout "120";'  >> /etc/apt/apt.conf.d/80retries && \
    echo 'Acquire::https::Timeout "120";' >> /etc/apt/apt.conf.d/80retries

# ============================================================
# Stage 3: System & ROS Binary Dependencies
# ============================================================

# 3.1 Base system tools
RUN apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
      wget ca-certificates unzip git \
      python3-pip python3-rosdep \
      curl gnupg2 \
      libusb-1.0-0-dev \
      usbutils \
      libgl1 \
      libegl1 \
      nano \
      htop \
      libxcb-shape0 \
      libgles2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3.2 ROS Jazzy binaries (with retries)
#     NOTE: ros-jazzy-fields2cover intentionally excluded here —
#     we build from source with BUILD_PYTHON=ON in Stage 3.5 instead.
RUN for i in 1 2 3 4 5; do \
      apt-get update && \
      apt-get install -y --fix-missing --no-install-recommends \
        libinput-bin \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-amcl \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-behavior-tree \
        ros-jazzy-nav2-bt-navigator \
        ros-jazzy-septentrio-gnss-driver \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-rtcm-msgs \
        ros-jazzy-nmea-msgs \
        ros-jazzy-libcurl-vendor \
        ros-jazzy-topic-tools \
        ros-jazzy-usb-cam \
        ros-jazzy-image-transport \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-tf-transformations \
        ros-jazzy-interactive-markers \
        ros-jazzy-control-msgs \
        ros-jazzy-controller-manager-msgs \
        ros-jazzy-rviz2 \
        python3-sympy \
        python3-fonttools \
        python3-ufolib2 \
        python3-matplotlib \
        python3-serial python3-requests python3-yaml && \
      break || (echo "Stage 3.2 attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 3.5: Fields2Cover — build from source with Python bindings
# ============================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      cmake \
      libgdal-dev \
      libeigen3-dev \
      libboost-all-dev \
      libtbb-dev \
      swig \
      python3-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

ARG F2C_REF=v2.0.0

RUN git clone --depth 1 --branch ${F2C_REF} \
      https://github.com/Fields2Cover/Fields2Cover.git /opt/Fields2Cover && \
    cmake -S /opt/Fields2Cover -B /opt/Fields2Cover/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PYTHON=ON \
      -DBUILD_TUTORIALS=OFF \
      -DBUILD_TESTS=OFF && \
    cmake --build /opt/Fields2Cover/build --parallel "$(nproc)" && \
    cmake --install /opt/Fields2Cover/build && \
    ldconfig && \
    SITE=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
    find /opt/Fields2Cover/build -name "_fields2cover*.so" -exec cp {} "$SITE/" \; && \
    find /opt/Fields2Cover/build -name "fields2cover.py"  -exec cp {} "$SITE/" \; && \
    rm -rf /opt/Fields2Cover/build

# Smoke-test: make sure `import fields2cover` works
RUN python3 -c "import fields2cover as f2c; print('fields2cover OK:', f2c.__version__)"

# ============================================================
# Stage 4: Lizard (ESP32 devtools)
# ============================================================
RUN GIT_TERMINAL_PROMPT=0 git clone --depth 1 \
        https://github.com/Agroecology-Lab/lizard /root/.lizard && \
    chmod +x /root/.lizard/espresso.py && \
    pip install --break-system-packages --ignore-installed --retries 10 \
      -r /root/.lizard/requirements.txt

# ============================================================
# Stage 5: Python Dependencies
# ============================================================

# 5.1 Project requirements.txt + headless OpenCV
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed \
      --retries 10 --timeout 1000 \
      opencv-python-headless \
      -r /tmp/requirements.txt

# 5.2 CPU-only PyTorch + YOLOX
RUN pip install --break-system-packages --no-cache-dir \
      "transforms3d>=0.4.2" && \
    pip install --break-system-packages --no-cache-dir \
      torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip install --break-system-packages --no-deps yolox==0.3.0 && \
    pip install --break-system-packages --no-cache-dir \
      thop tqdm loguru ninja tabulate \
      nicegui pyserial lizard pymongo networkx

# 5.3 YOLOX-Nano pretrained weights
RUN mkdir -p /root/models && \
    wget -q -O /root/models/yolox_nano.pth \
      https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.pth

# ============================================================
# Stage 6: Shell Environment
# ============================================================
COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

RUN mkdir -p /root/.rosys && echo '{}' > /root/.rosys/rosys.config.json

ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages"
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3

# ============================================================
# Stage 7: Workspace — Local Sources
# ============================================================
WORKDIR /workspace
COPY ./src/devkit_driver    ./src/devkit_driver
COPY ./src/devkit_launch    ./src/devkit_launch
COPY ./src/devkit_ui        ./src/devkit_ui
COPY /sowbot_row_follow ./src/sowbot_row_follow

# ============================================================
# Stage 8: Workspace — Git Clones (each repo has its own
#           retry loop so one failure does not skip the rest)
# ============================================================
RUN for i in 1 2 3 4 5; do \
      GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b main \
        https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss && break || \
      (echo "ublox_dgnss attempt $i failed, retrying in 15s..." && sleep 15); \
    done

RUN sed -i 's/default_value="Test Rover"/default_value=""/' \
      src/ublox_dgnss/ublox_dgnss/launch/ublox_mb+r_rover.launch.py

RUN for i in 1 2 3 4 5; do \
      GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b aoc_refactor \
        https://github.com/LCAS/topological_navigation src/topological_navigation && break || \
      (echo "topological_navigation attempt $i failed, retrying in 15s..." && sleep 15); \
    done

RUN for i in 1 2 3 4 5; do \
      GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b main \
        https://github.com/samuk/caatingarobotics src/caatingarobotics && break || \
      (echo "caatingarobotics attempt $i failed, retrying in 15s..." && sleep 15); \
    done

RUN for i in 1 2 3 4 5; do \
      GIT_TERMINAL_PROMPT=0 git clone --depth 1 \
        https://github.com/manankharwar/fusioncore src/fusioncore && break || \
      (echo "fusioncore attempt $i failed, retrying in 15s..." && sleep 15); \
    done

RUN for i in 1 2 3 4 5; do \
      GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b master \
        https://github.com/LCAS/sentor src/sentor && break || \
      (echo "sentor attempt $i failed, retrying in 15s..." && sleep 15); \
    done

# ============================================================
# Stage 9: rosdep — Automated Dependency Resolution
# ============================================================
RUN apt-get update && \
    apt-get install -y --fix-broken && \
    if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && \
    rosdep update && \
    for i in 1 2 3 4 5; do \
      apt-get update && \
      rosdep install --from-paths src --ignore-src -y \
        --rosdistro jazzy \
        --skip-keys "\
          fusioncore \
          libopencv-dev \
          opencv-python-headless \
          PyQt5 PyQtWebEngine \
          ros-jazzy-cv-bridge \
          ros-jazzy-compressed-image-transport \
          ros-jazzy-compressed-depth-image-transport \
          ros-jazzy-theora-image-transport \
          ros-jazzy-nav2-behavior-tree \
          ros-jazzy-nav2-bt-navigator \
          ros-jazzy-tf-transformations \
          ros-jazzy-nav2-bringup \
          ros-jazzy-navigation2 \
          ros-jazzy-opennav-docking \
          ros-jazzy-fields2cover \
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
          yolox \
          sentor" && \
      break || (echo "rosdep attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 10: colcon Build
# ============================================================
RUN apt-get update && \
    apt-get install -f -y && \
    for i in 1 2 3 4 5; do \
      apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rtcm-msgs \
        ros-jazzy-nmea-msgs \
        ros-jazzy-libcurl-vendor \
        ros-jazzy-interactive-markers \
        ros-jazzy-tf-transformations \
        ros-jazzy-rviz2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
        ros-jazzy-topic-tools \
        usbutils libusb-1.0-0-dev && \
      break || (echo "Stage 10 apt attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    pip install --break-system-packages --no-cache-dir transforms3d && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 11: Runtime Environment
# ============================================================
# RMW set after build — setting before colcon can disrupt DDS discovery
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["/bin/bash", "-c", \
  "source /opt/ros/jazzy/setup.bash && \
  [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash && \
  exec \"$@\"", "--"]
CMD ["bash"]
