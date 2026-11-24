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
exec /usr/bin/python3 tools/demo.py
