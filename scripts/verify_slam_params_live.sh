#!/usr/bin/env bash
# verify_slam_params_live.sh
# Run while slam_mapping.launch.py is active to confirm the fix is live.
#
# Usage:
#   cd ~/butler_robot_ws && source install/setup.bash
#   bash scripts/verify_slam_params_live.sh

echo "=========================================="
echo " LIVE SLAM_TOOLBOX PARAMETER CHECK"
echo "=========================================="
echo ""
echo "Querying the running slam_toolbox node directly..."
echo ""

PASS=1
check_param() {
    local param=$1
    local expected=$2
    local actual
    actual=$(ros2 param get /slam_toolbox "$param" 2>&1 | grep -oP '[\d.]+' | head -1)
    if [ "$actual" = "$expected" ]; then
        echo "  OK  $param = $actual"
    else
        echo "  FAIL $param = $actual  (expected $expected)"
        PASS=0
    fi
}

check_param "minimum_travel_distance" "0.2"
check_param "minimum_travel_heading"  "0.2"
check_param "loop_match_minimum_response_coarse" "0.45"
check_param "loop_match_minimum_response_fine"   "0.55"
check_param "scan_buffer_size" "14"
check_param "tf_buffer_duration" "30.0"

echo ""
if [ "$PASS" = "1" ]; then
    echo "ALL PARAMS CORRECT — fix is live. Safe to start driving."
else
    echo "SOME PARAMS WRONG — fix did NOT reach the running process."
    echo ""
    echo "Run:"
    echo "  rm -rf ~/butler_robot_ws/build ~/butler_robot_ws/install ~/butler_robot_ws/log"
    echo "  cd ~/butler_robot_ws && colcon build --symlink-install"
    echo "  source install/setup.bash"
    echo "  ros2 launch butler_bringup slam_mapping.launch.py"
fi
echo "=========================================="
