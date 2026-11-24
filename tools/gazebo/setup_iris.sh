#!/usr/bin/env bash
# Prepare the Gazebo Classic flight dynamics plugin and Iris quadrotor locally.
set -euo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
plugin="$workspace/runtime/gazebo-classic-plugin"
revision=a28cab40f939a42d4845390ed5ace6d36618385c
firmware_revision=d9c5097c3d8333435712ffed86bca8ce00c4bb46

mkdir -p "$workspace/runtime"
touch "$workspace/runtime/COLCON_IGNORE"
if [[ ! -d "$plugin/.git" ]]; then
    git clone --depth 1 https://github.com/khancyr/ardupilot_gazebo.git "$plugin"
fi
if [[ $(git -C "$plugin" rev-parse HEAD) != "$revision" ]]; then
    echo "Expected the archived Gazebo Classic plugin at revision $revision in $plugin" >&2
    exit 1
fi
if [[ ! -f "$plugin/build/libArduPilotPlugin.so" ]]; then
    cmake -S "$plugin" -B "$plugin/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$plugin/build" --target ArduPilotPlugin -j2
fi

# The upstream SDF includes a separately downloaded optional gimbal. Use the
# supplied airframe and rotors, and express the Gazebo world in local ENU.
python3 - "$plugin" <<'PY'
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

root = pathlib.Path(sys.argv[1])
relative = 'models/iris_with_ardupilot/model.sdf'
original = subprocess.check_output(['git', '-C', str(root), 'show', 'HEAD:' + relative])
sdf = ET.fromstring(original)
model = sdf.find('model')
for include in model.findall('include'):
    if include.findtext('uri') == 'model://gimbal_small_2d':
        model.remove(include)
model.remove(model.find("joint[@name='iris_gimbal_mount']"))
plugin = model.find("plugin[@filename='libArduPilotPlugin.so']")
plugin.find('gazeboXYZToNED').text = '0 0 0 3.14159265359 0 1.57079632679'
(root / relative).write_bytes(ET.tostring(sdf, encoding='utf-8', xml_declaration=True))
PY

defaults="$plugin/gazebo-iris.parm"
checksum=4aee140cbb29b967e4519e4c84c4b389a2172c30e542f36b4fba4c118b4d4750
if ! printf '%s  %s\n' "$checksum" "$defaults" | sha256sum --check --status 2>/dev/null; then
    tmp=$(mktemp "$plugin/.iris-parm.XXXXXX")
    trap 'rm -f -- "$tmp"' EXIT
    url="https://raw.githubusercontent.com/ArduPilot/ardupilot/$firmware_revision/Tools/autotest/default_params/gazebo-iris.parm"
    curl --fail --silent --show-error --location --retry 3 "$url" -o "$tmp"
    printf '%s  %s\n' "$checksum" "$tmp" | sha256sum --check --status
    mv -- "$tmp" "$defaults"
fi
printf 'Gazebo Classic Iris model and flight dynamics plugin are ready in %s\n' "$plugin"
