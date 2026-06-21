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

# NOTE: This image runs on NumPy 2.x image-wide.
# rosys (via nicegui) requires numpy>=1.26 with no upper bound, and
# opencv-python-headless>=4.12 hard-requires numpy>=2,<2.3 — so numpy<2 is
# not an option once rosys is present. The cost is that the apt-prebuilt
# cv_bridge is compiled against NumPy 1.x and would crash under NumPy 2;
# we therefore rebuild cv_bridge (vision_opencv) from source in Stage 10,
# after numpy 2 is installed, and remove the apt build so it can't shadow it.

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
    apt-get install -y --no-install-recommends \
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
      libgles2 \
    || (apt-get update && apt-get install -y --no-install-recommends \
      wget ca-certificates unzip git python3-pip python3-rosdep \
      curl gnupg2 libusb-1.0-0-dev usbutils libgl1 libegl1 nano htop \
      libxcb-shape0 libgles2) && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


# 3.2 ROS Jazzy binaries (with retries)
#     NOTE: ros-jazzy-fields2cover intentionally excluded here —
#     we build from source with BUILD_PYTHON=ON in Stage 3.5 instead.
#     NOTE: ros-jazzy-cv-bridge intentionally NOT installed here —
#     it is built from source in Stage 10 against NumPy 2.
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
        ros-jazzy-web-video-server \
        python3-matplotlib \
        python3-serial python3-requests python3-yaml && \
      break || (echo "Stage 3.2 attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 3.5: Fields2Cover — build from source with Python bindings
# ============================================================
RUN apt-get update && \
    apt --fix-broken install -y && \
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
      shapely \
      -r /root/.lizard/requirements.txt
# ============================================================
# Stage 5: Python Dependencies
# ============================================================

# 5.1 Project requirements.txt + headless OpenCV
#     opencv-python-headless 4.12.0.88 ships as a cp37-abi3 manylinux wheel
#     (no sdist build) and requires numpy>=2,<2.3 — fine now the image is on
#     numpy 2. Matches the opencv-contrib-python pin rosys carries.
#     IMPORTANT: requirements.txt must NOT pin numpy<2 anymore. If it still
#     contains `numpy<2.0.0`, change it to `numpy>=2,<2.3` or remove the line.
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages --ignore-installed \
      --retries 10 --timeout 1000 \
      "opencv-python-headless==4.12.0.88" \
      -r /tmp/requirements.txt

# 5.2 CPU-only PyTorch + YOLOX
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages "transforms3d>=0.4.2" && \
    pip install --break-system-packages \
      torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip install --break-system-packages --no-deps yolox==0.3.0 && \
    pip install --break-system-packages \
      thop tqdm loguru ninja tabulate \
      nicegui pyserial lizard pymongo networkx

# 5.3 YOLOX-Nano pretrained weights
RUN mkdir -p /root/models && \
    wget -q -O /root/models/yolox_nano.pth \
      https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.pth

# 5.4 Forest3D — realistic field generation
RUN for i in 1 2 3 4 5; do \
      GIT_TERMINAL_PROMPT=0 git clone --depth 1 -b feature/terrain-types-refactor \
        https://github.com/unitsSpaceLab/Forest3D.git src/forest3d && break || \
      (echo "Forest3D attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    pip install --break-system-packages --no-cache-dir \
      src/forest3d && \
    rm -rf src/forest3d/.git

COPY docker/forest3d-models /workspace/models

# ============================================================
# Stage 6: Shell Environment
# ============================================================

COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

RUN mkdir -p /root/.rosys && echo '{}' > /root/.rosys/rosys.config.json

ENV PYTHONPATH="/root/.lizard:/workspace/install/lib/python3.12/site-packages"

# ENV MESA_GL_VERSION_OVERRIDE=3.3 commented out for testing, remove if no issues found

# ============================================================
# Stage 7: Workspace dir
# ============================================================

WORKDIR /workspace

# ============================================================
# Stage 8: Workspace — Git Clones (each repo has its own
#           retry loop so one failure does not skip the rest)
# ============================================================
# Placed BEFORE the local-source COPY (Stage 7.5) so that editing local code
# (devkit_driver/devkit_bringup/devkit_ui/devkit_simulation) does NOT invalidate
# this layer and force a re-clone of every external repo. Docker only
# invalidates a RUN layer if its own inputs change, and this stage's only
# input is EXTERNAL_CACHEBUST below — bump that (via `manage.py build +pull`)
# to force a fresh pull of just these repos, without a full --no-cache build.
ARG EXTERNAL_CACHEBUST=1

# All repos clone in parallel; each retries 5x. Any failed clone fails the
# build (a missing repo breaks rosdep/colcon downstream anyway).
RUN clone() { \
      for i in 1 2 3 4 5; do \
        GIT_TERMINAL_PROMPT=0 git clone --depth 1 "$@" && return 0; \
        echo "clone $* attempt $i failed, retrying in 15s..."; sleep 15; \
      done; echo "FATAL: clone failed: $*"; return 1; \
    }; \
    pids=""; \
    clone -b main      https://github.com/aussierobots/ublox_dgnss src/ublox_dgnss & pids="$pids $!"; \
    clone -b 4.1.0     https://github.com/ros-perception/vision_opencv.git src/vision_opencv & pids="$pids $!"; \
    clone -b aoc_refactor https://github.com/LCAS/topological_navigation src/topological_navigation & pids="$pids $!"; \
    clone -b main      https://github.com/samuk/caatingarobotics src/caatingarobotics & pids="$pids $!"; \
    clone              https://github.com/manankharwar/fusioncore src/fusioncore & pids="$pids $!"; \
    clone -b ros2-gz   https://github.com/FieldRobotEvent/virtual_maize_field src/virtual_maize_field & pids="$pids $!"; \
    clone -b master    https://github.com/LCAS/sentor src/sentor & pids="$pids $!"; \
    clone -b main      https://github.com/nilseuropa/ros2graph_explorer src/ros2graph_explorer & pids="$pids $!"; \
    clone -b main      https://github.com/Supull/ros2grapher.git src/ros2grapher & pids="$pids $!"; \
    rc=0; for p in $pids; do wait "$p" || rc=1; done; \
    test "$rc" = 0 && test -f src/vision_opencv/cv_bridge/package.xml

RUN sed -i 's/default_value="Test Rover"/default_value=""/' \
      src/ublox_dgnss/ublox_dgnss/launch/ublox_mb+r_rover.launch.py

# Patch fusioncore: BEST_EFFORT QoS for /gnss/fix (ublox publishes BEST_EFFORT;
# fusioncore's `10` shorthand is RELIABLE → never connects). grep halts the
# build if upstream changes the subscription form.
RUN awk '/create_subscription<sensor_msgs::msg::NavSatFix>/ { in_sub=1 } in_sub && /, 10,/ { sub(/, 10,/, ", rclcpp::SensorDataQoS(),"); in_sub=0 } { print }' \
      src/fusioncore/fusioncore_ros/src/fusion_node.cpp > /tmp/_fc.cpp && \
    mv /tmp/_fc.cpp src/fusioncore/fusioncore_ros/src/fusion_node.cpp && \
    grep -q "rclcpp::SensorDataQoS()" src/fusioncore/fusioncore_ros/src/fusion_node.cpp

# ============================================================
# Stage 7.5: Workspace — Local Sources
# ============================================================
# Now AFTER the external clones (Stage 8). A local code edit only
# invalidates this layer and what follows (colcon build) — not the
# git-clone layer above, which stays cached.
COPY ./src/devkit_driver    ./src/devkit_driver
COPY ./src/devkit_bringup    ./src/devkit_bringup
COPY ./src/devkit_ui        ./src/devkit_ui
COPY ./src/devkit_simulation   ./src/devkit_simulation

# devkit_simulation now lives in this repo (src/devkit_simulation, copied above).
# Drop the copy that ships inside caatingarobotics so colcon doesn't see two.
RUN rm -rf src/caatingarobotics/src/agro_robot_sim

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
          ros-jazzy-rqt-image-view \
          cv_bridge \
          image_geometry \
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
          devkit_simulation \
          caatinga_nav \
          caatinga_vision \
          sowbot_row_follow \
          yolox \
          sentor" && \
      break || (echo "rosdep attempt $i failed, retrying in 15s..." && sleep 15); \
    done && \
    rm -rf /var/lib/apt/lists/*

ARG CACHEBUST=1  ## Move this line earlier in the file if you want to start the build process earlier

# ============================================================
# Stage 10: colcon Build
# ============================================================
# cv_bridge / image_geometry come from src/vision_opencv and are built here,
# AFTER numpy 2 is in place, so the boost extension links against numpy 2's
# headers. Any apt-prebuilt cv_bridge is removed first so its numpy-1.x .so
# cannot shadow the source build on the ament prefix.
RUN apt-get update && \
    apt-get install -f -y && \
    for i in 1 2 3 4 5; do \
      apt-get install -y --no-install-recommends \
        ros-jazzy-web-video-server \
        xvfb \
        x11vnc \
        websockify \
        novnc \
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
    # Remove only the cv_bridge/image_geometry FILES (numpy-1.x .so + python),
    # NOT the apt packages — `apt-get remove ros-jazzy-cv-bridge` cascades into
    # usb_cam, navigation2, nav2-bringup and the image-transport plugins, which
    # then never come back. The source build in /workspace/install overlays the
    # base on AMENT_PREFIX_PATH anyway; we just delete the stale numpy-1.x .so
    # and python module from the base prefix so they can't be imported first.
    rm -rf /opt/ros/jazzy/lib/python3.12/site-packages/cv_bridge \
           /opt/ros/jazzy/lib/python3.12/site-packages/image_geometry \
           /opt/ros/jazzy/lib/python3.12/site-packages/cv_bridge-*.dist-info \
           /opt/ros/jazzy/lib/libcv_bridge.so \
           /opt/ros/jazzy/lib/libimage_geometry.so 2>/dev/null || true && \
    pip install --break-system-packages --no-cache-dir transforms3d && \
    . /opt/ros/jazzy/setup.sh && \
    MAKEFLAGS="-j$(nproc)" colcon build --symlink-install \
      --parallel-workers "$(nproc)" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Smoke-test: cv_bridge must import cleanly under numpy 2 (the crop_row_node fix)
RUN . /opt/ros/jazzy/setup.sh && \
    . /workspace/install/setup.sh && \
    python3 -c "import numpy; from cv_bridge import CvBridge; CvBridge(); print('cv_bridge OK on numpy', numpy.__version__)"

ARG INSTALL_SIM=false
RUN if [ "$INSTALL_SIM" = "true" ]; then \
      apt-get update && apt-get install -y \
        gz-harmonic ros-jazzy-ros-gz-sim && \
      rm -rf /var/lib/apt/lists/*; \
    fi
# NOTE: both hardware/neo and sim images now run NumPy 2.x. gz-harmonic also
# requires numpy>=2, so the previous per-image numpy split is gone — the only
# difference INSTALL_SIM makes is adding gz-harmonic + ros-gz-sim.

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
