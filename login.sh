#!/bin/bash
# Search for any active container matching our naming conventions
CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep -E '^feldfreund_runtime$|^feldfreund_dev$|^open_ag_runtime$|^sowbot_runtime$|^open_agbot$' | head -n 1)
if [ -z "$CONTAINER_NAME" ]; then
    echo "------------------------------------------------------"
    echo "Error: No running Feldfreund container found."
    echo "Ensure you have started the stack using ./manage.py"
    echo "------------------------------------------------------"
    exit 1
fi
echo "------------------------------------------------------"
echo "Found Container: $CONTAINER_NAME"
echo "Entering shell environment..."
echo "------------------------------------------------------"

# DDS config MUST match what manage.py injects at `docker run` time, otherwise
# this exec'd shell (and the ros2 daemon it spawns) discovers a different graph
# than the running stack — the classic "empty `ros2 node list` while neo is up".
# `docker exec` does NOT inherit `docker run --env`, so we set it explicitly.
#
# Strategy: read CYCLONEDDS_URI directly from the running container's env.
# manage.py already computed the right config (loopback for sim/dev, crossover
# peer-to-peer for on-robot 192.168.10.x) and passed it via --env at docker run.
# Reading it back here is simpler and more robust than duplicating that logic,
# and avoids any hardcoded-XML quoting bugs.
DDS_URI=$(docker exec "$CONTAINER_NAME" printenv CYCLONEDDS_URI 2>/dev/null)

if [ -z "$DDS_URI" ]; then
    # Container was started without CYCLONEDDS_URI (shouldn't happen with a
    # current manage.py, but fall back gracefully to the loopback config).
    echo "[WARN] CYCLONEDDS_URI not found in container env — falling back to loopback."
    DDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>'
fi

# Enter the container and source the environment
# Handles both Humble and Jazzy paths automatically
docker exec -it \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI="$DDS_URI" \
    "$CONTAINER_NAME" bash -c "
    source /opt/ros/*/setup.bash 2>/dev/null
    if [ -f /workspace/install/setup.bash ]; then
        source /workspace/install/setup.bash
    fi
    export PYTHONPATH=\$PYTHONPATH:/workspace/src/devkit_ui
    # Bounce the ros2 daemon so it rediscovers the currently-running stack under
    # the DDS config above, instead of serving a graph it cached earlier (before
    # the stack was up, or under a different URI). One restart per shell entry —
    # NOT in bashrc, which would thrash on every subshell.
    ros2 daemon stop >/dev/null 2>&1
    ros2 daemon start >/dev/null 2>&1
    bash
"
