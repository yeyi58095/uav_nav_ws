#!/usr/bin/env bash
set -euo pipefail

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
runtime="$workspace/runtime"
firmware="$runtime/firmware"
version=4.7.3
revision=d9c5097c3d8333435712ffed86bca8ce00c4bb46

if [[ $(uname -sm) != 'Linux x86_64' ]]; then
    echo 'This ArduCopter SITL binary requires x86_64 Linux.' >&2
    exit 1
fi

mkdir -p "$firmware"
touch "$runtime/COLCON_IGNORE"
download=$(mktemp "$firmware/.download.XXXXXX")
trap 'rm -f -- "$download"' EXIT

fetch_verified() {
    local url=$1 file=$2 checksum=$3
    if [[ -f "$file" ]] && printf '%s  %s\n' "$checksum" "$file" | sha256sum --check --status; then
        return
    fi
    curl --fail --location --silent --show-error --retry 3 "$url" --output "$download"
    printf '%s  %s\n' "$checksum" "$download" | sha256sum --check --status || {
        echo "Checksum mismatch for $url" >&2
        return 1
    }
    mv -f -- "$download" "$file"
}

fetch_verified \
    "https://firmware.ardupilot.org/Copter/stable-$version/SITL_x86_64_linux_gnu/arducopter" \
    "$firmware/arducopter" 0cab7ddc9e9f1ef7f7c88138fcf925824fb952eb15e4f69cd6c8ef9210e25bf0
chmod +x "$firmware/arducopter"
fetch_verified \
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/$revision/Tools/autotest/default_params/copter.parm" \
    "$firmware/copter.parm" 5e01345b45d1c6190b28bece5638bbdd4cf1cce35e05bbbf480ab24d2b51aa0e

if [[ ! -x "$runtime/venv/bin/python" ]]; then
    python3 -m venv --system-site-packages "$runtime/venv"
fi
if ! "$runtime/venv/bin/python" -c 'from importlib.metadata import version; from pymavlink import mavutil; assert version("pymavlink") == "2.4.49"' 2>/dev/null; then
    "$runtime/venv/bin/python" -m pip install --disable-pip-version-check 'pymavlink==2.4.49'
fi
"$runtime/venv/bin/python" -c 'from pymavlink import mavutil'
"$firmware/arducopter" --help >/dev/null
printf 'ArduCopter %s and pymavlink are ready in %s\n' "$version" "$runtime"
