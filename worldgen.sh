#!/bin/bash
# Shared world generator used by BOTH the Launch Sim button
# (sowbot_sim.launch.py world_gen step) and the Rebuild button
# (ui_node.py _rebuild_world). Single source of truth for:
#   - reading the UI's placement settings (settings: block in forest3d.yaml)
#   - the input-keyed cache (skip regeneration when nothing changed)
#   - actually running topo_to_forest3d.py to write maize.world
#
# Settings can be passed as CLI args (as the Rebuild button does, forwarding
# current UI values) or left unset to fall back to the settings: block of the
# last forest3d.yaml. Either way the resolved values feed the cache key, so
# Rebuild and Launch Sim agree on when a regeneration is truly needed.
#
# The cache key hashes: resolved settings, the topo map, crop/weed model
# files, uploaded terrain, and the generator script itself. The key file
# lives beside the world in install/ (same container lifetime), so a fresh
# `manage.py up --sim` always regenerates once; afterwards Launch Sim is
# instant until the user changes settings/models/terrain.

set -u

# Headland margin is a fixed world-layout constant, not user-configurable.
# 5.0m: row_discovery_node's own worst-case straight-line excursion past a
# row end is headland_clearance_m (0.5m) + max_search_distance_m (1.5m) =
# 2.0m — this must stay comfortably UNDER the world's actual headland
# margin, or discovery mode drives the robot off the edge. If either number
# changes, check the other.
HEADLAND=5.0

PS_ARG=""; RW_ARG=""; SC_ARG=""; CAT_ARG=""; MODEL_ARG=""; WD_ARG=""
while [ "$#" -gt 0 ]; do
    case "$1" in
        --plant-spacing) PS_ARG="$2"; shift 2 ;;
        --row-width)     RW_ARG="$2"; shift 2 ;;
        --plant-scale)   SC_ARG="$2"; shift 2 ;;
        --scale-category) CAT_ARG="$2"; shift 2 ;;
        --crop-model)    MODEL_ARG="$2"; shift 2 ;;
        --weed-density)  WD_ARG="$2"; shift 2 ;;
        *)
            echo "[worldgen] unknown arg: $1" >&2; exit 2 ;;
    esac
done

WORLD=/workspace/install/devkit_simulation/share/devkit_simulation/worlds/maize.world
KEYFILE="$WORLD.key"

SET=$(grep -A7 "^settings:" /workspace/forest3d.yaml 2>/dev/null || true)
get() { echo "$SET" | grep -m1 -E "^  $1:" | awk '{print $2}'; }
PS=${PS_ARG:-$(get plant_spacing)};     [ -n "$PS" ]    || PS=1.0
RW=${RW_ARG:-$(get row_width)};         [ -n "$RW" ]    || RW=0.9
SC=${SC_ARG:-$(get plant_scale)};       [ -n "$SC" ]    || SC=1.0
CAT=${CAT_ARG:-$(get scale_category)};  [ -n "$CAT" ]   || CAT=all
MODEL=${MODEL_ARG:-$(get crop_model)};  [ -n "$MODEL" ] || MODEL=plant
WD=${WD_ARG:-$(get weed_density)};      [ -n "$WD" ]    || WD=10

KEY=$(
    {
        echo "PS=$PS RW=$RW SC=$SC CAT=$CAT MODEL=$MODEL WD=$WD"
        md5sum /workspace/maps/maize_map
        find /workspace/models/crop /workspace/models/weed \
            -type f \( -name "*.sdf" -o -name "*.glb" -o -name "*.stl" \) \
            -exec md5sum {} + 2>/dev/null
        find /workspace/uploads/terrain -type f -exec md5sum {} + 2>/dev/null
        find /workspace/uploads/soil_custom/textures -type f -exec md5sum {} + 2>/dev/null
        md5sum /workspace/topo_to_forest3d.py
    } | md5sum | awk '{print $1}'
)

LOCKFILE="$WORLD.lock"
mkdir -p "$(dirname "$WORLD")"
exec 9>"$LOCKFILE"
flock 9

if [ -f "$WORLD" ] && [ -f "$KEYFILE" ] && [ "$KEY" = "$(cat "$KEYFILE")" ]; then
    echo "[world] up-to-date — skipping regeneration"
    exit 0
fi

echo "[world] regenerating..."
python3 /workspace/topo_to_forest3d.py \
    --topo /workspace/maps/maize_map \
    --out /workspace/forest3d.yaml \
    --headland "$HEADLAND" --generate \
    --world-out "$WORLD" \
    --models-path /workspace/models \
    --plant-spacing "$PS" \
    --row-width "$RW" \
    --plant-scale "$SC" \
    --scale-category "$CAT" \
    --crop-model "$MODEL" \
    --weed-density "$WD" || exit 1
echo "$KEY" > "$KEYFILE"
