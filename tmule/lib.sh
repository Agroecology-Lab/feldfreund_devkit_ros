# Shared shell helpers for the TMuLE sim configs.
#
# Sourced by each config's `init_cmd`, so every pane (and every `check`) gets
# these functions. The whole sim stack runs against ONE Docker container
# started by `./manage.py --sim`; the gazebo/crop panes just `docker exec` into
# it, so they must wait for it (and for the generated world) to be ready.
#
# Timeouts are generous on purpose: the nav_stack pane may sit on a `sudo`
# password prompt (fixusb.py) and a first-ever run also generates the world.
# Override via env if needed, e.g. FF_WAIT_CONTAINER_TIMEOUT=120.

# Name of the running runtime container. Mirrors the detection in login.sh so
# this keeps working across the project's historical container names.
ff_container() {
    docker ps --format '{{.Names}}' \
        | grep -E '^(sowbot_runtime|feldfreund_runtime|feldfreund_dev|open_ag_runtime|open_agbot)$' \
        | head -n1
}

# Block until the runtime container exists. Returns non-zero on timeout.
ff_wait_container() {
    cn=""
    i=0
    timeout="${FF_WAIT_CONTAINER_TIMEOUT:-600}"
    echo "[tmule] waiting up to ${timeout}s for the runtime container"
    echo "[tmule]   (nav_stack pane runs ./manage.py --sim — it may ask for a sudo password)"
    while [ "$i" -lt "$timeout" ]; do
        cn="$(ff_container)"
        if [ -n "$cn" ]; then
            echo "[tmule] runtime container is up: $cn"
            return 0
        fi
        i=$((i + 1))
        [ $((i % 30)) -eq 0 ] && echo "[tmule] ...still waiting for the container (${i}s)"
        sleep 1
    done
    echo "[tmule] ERROR: runtime container did not appear within ${timeout}s" >&2
    return 1
}

# Block until the topo-derived Gazebo world has been generated inside the
# container. Warns and returns 0 (does NOT block launch) if it never shows up,
# so Gazebo still gets a chance to start.
ff_wait_world() {
    world="/workspace/install/devkit_simulation/share/devkit_simulation/worlds/maize.world"
    cn="$(ff_container)"
    [ -z "$cn" ] && return 0
    i=0
    timeout="${FF_WAIT_WORLD_TIMEOUT:-600}"
    echo "[tmule] waiting up to ${timeout}s for the generated world ($world)"
    while [ "$i" -lt "$timeout" ]; do
        if docker exec "$cn" test -f "$world" 2>/dev/null; then
            echo "[tmule] world is ready"
            return 0
        fi
        i=$((i + 2))
        sleep 2
    done
    echo "[tmule] WARN: world not detected after ${timeout}s; launching Gazebo anyway" >&2
    return 0
}

# Run a ROS 2 command inside the runtime container with the environment sourced
# (mirrors login.sh and manage.py's _ros_source). Usage: ff_exec ros2 launch ...
ff_exec() {
    cn="$(ff_container)"
    if [ -z "$cn" ]; then
        echo "[tmule] ERROR: no runtime container found; is ./manage.py --sim running?" >&2
        return 1
    fi
    docker exec -it "$cn" bash -lc '
        source /opt/ros/*/setup.bash 2>/dev/null
        [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
        export PYTHONPATH="$PYTHONPATH:/workspace/src/devkit_ui"
        exec "$@"' bash "$@"
}
