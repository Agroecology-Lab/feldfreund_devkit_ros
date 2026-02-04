#!/bin/bash

# Search for any active container matching our naming conventions
CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep -E '^feldfreund_runtime$|^feldfreund_dev$|^open_ag_runtime$|^open_agbot$' | head -n 1)

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

# Enter the container and source the environment
# Handles both Humble and Jazzy paths automatically
docker exec -it "$CONTAINER_NAME" bash -c "
    source /opt/ros/*/setup.bash 2>/dev/null
    if [ -f /workspace/install/setup.bash ]; then
        source /workspace/install/setup.bash
    fi
    export PYTHONPATH=\$PYTHONPATH:/workspace/src/devkit_ui
    bash
"
