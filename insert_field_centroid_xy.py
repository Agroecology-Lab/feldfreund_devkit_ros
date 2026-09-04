import pathlib
import sys

path = pathlib.Path.home() / 'feldfreund_devkit_ros/src/devkit_f2c_planner/devkit_f2c_planner/f2c_planner.py'
text = path.read_text()

if 'def field_centroid_xy' in text:
    print(f'Already present in {path} — doing nothing.')
    sys.exit(0)

anchor = (
    "def _f2c_xy_to_latlon(x: float, y: float,\n"
    "                      lat0: float, lon0: float) -> tuple[float, float]:\n"
    "    R   = 6_378_137.0\n"
    "    lat = lat0 + math.degrees(y / R)\n"
    "    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))\n"
    "    return lat, lon\n"
)

if anchor not in text:
    print('ERROR: anchor text not found — file has diverged more than expected.')
    print('Not touching the file. Paste this script\'s output back for a look.')
    sys.exit(1)

insertion = (
    "\n\n"
    "def field_centroid_xy(corners_ll: list) -> tuple[float, float]:\n"
    "    \"\"\"Field boundary's spatial centroid, in the same local-xy frame\n"
    "    _run_f2c()/_run_contour_f2c() use (anchored at corners_ll[0]).\n"
    "\n"
    "    Callers pass this straight through as dem.select_reference_contour_xy()'s\n"
    "    centroid_xy argument, to get a reference contour centred on the field —\n"
    "    see that function's docstring for why centring keeps drift symmetric.\n"
    "    \"\"\"\n"
    "    lat0, lon0 = corners_ll[0]\n"
    "    poly = Polygon([_f2c_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in corners_ll])\n"
    "    if not poly.is_valid:\n"
    "        poly = poly.buffer(0)\n"
    "    return poly.centroid.x, poly.centroid.y\n"
)

new_text = text.replace(anchor, anchor + insertion, 1)
path.write_text(new_text)
print(f'Inserted field_centroid_xy into {path}')
