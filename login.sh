#!/bin/bash

# Updated to include 'open_ag_runtime'
CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep -E '^open_ag_runtime$|^open_agbot$|^open_ag_debug$' | head -n 1)

if [ -z "$CONTAINER_NAME" ]; then
    echo "------------------------------------------------------"
    echo "❌ ERROR: No running AgBot container found."
    echo "Container name 'open_ag_runtime' not found in 'docker ps'."
    echo "------------------------------------------------------"
    exit 1
fi

echo "------------------------------------------------------"
echo "✅ Found AgBot Container: $CONTAINER_NAME"
echo "🚀 Entering Bash environment..."
echo "------------------------------------------------------"

# Enter the container. 
# Note: Changed /open_agbot_ws to /workspace to match your manage.py volumes
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash; export PYTHONPATH=\$PYTHONPATH:/workspace/src/basekit_ui && bash"
