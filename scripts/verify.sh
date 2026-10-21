#!/usr/bin/env bash
set -euo pipefail
# Source the chosen ROS underlay before invoking this script.
source scripts/ros_env.sh
# Keep the test stack separate from the operator's ordinary ROS domain.
export ROS_DOMAIN_ID="${ATERKIA_TEST_DOMAIN_ID:-77}"
colcon build --symlink-install --parallel-workers 2
set +u
source install/setup.bash
set -u
colcon test --event-handlers console_direct+
colcon test-result --verbose
