#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
bash tools/setup_sitl.sh
source /opt/ros/foxy/setup.bash
colcon build --base-paths src --packages-select uav_nav --symlink-install
source install/local_setup.bash
site_dir=$(runtime/venv/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
export PYTHONPATH="$site_dir:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-44}
export ROS_LOCALHOST_ONLY=1
if [[ ${1:-} == --gazebo-camera ]]; then
    bash tools/gazebo/setup_iris.sh
    source /usr/share/gazebo-11/setup.sh
    export GAZEBO_MODEL_PATH="$PWD/runtime/gazebo-classic-plugin/models:${GAZEBO_MODEL_PATH:-}"
    export GAZEBO_PLUGIN_PATH="$PWD/runtime/gazebo-classic-plugin/build:${GAZEBO_PLUGIN_PATH:-}"
    export GAZEBO_MODEL_DATABASE_URI=""
    export GAZEBO_MASTER_URI="${GAZEBO_FLIGHT_MASTER_URI:-http://127.0.0.1:11445}"
fi
exec /usr/bin/python3 tools/demo.py "$@"
