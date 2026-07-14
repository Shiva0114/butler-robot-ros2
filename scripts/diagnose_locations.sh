#!/usr/bin/env bash
# diagnose_locations.sh
# ======================
# Directly inspects every copy of locations.yaml on disk to find out
# exactly which version is stale, instead of guessing.
#
# Usage: bash scripts/diagnose_locations.sh
# Run from the butler_robot_ws root.

set -e

echo "=========================================="
echo " LOCATIONS.YAML DIAGNOSTIC"
echo "=========================================="

SRC_FILE="src/butler_navigation/config/locations.yaml"
INSTALL_FILE="install/butler_navigation/share/butler_navigation/config/locations.yaml"

echo ""
echo "--- 1. SOURCE FILE ---"
if [ -f "$SRC_FILE" ]; then
    echo "Path: $SRC_FILE"
    echo "Top-level key:"
    head -1 "$SRC_FILE"
else
    echo "MISSING: $SRC_FILE"
fi

echo ""
echo "--- 2. INSTALLED FILE (what the launch file actually loads) ---"
if [ -f "$INSTALL_FILE" ]; then
    echo "Path: $INSTALL_FILE"
    if [ -L "$INSTALL_FILE" ]; then
        echo "Type: SYMLINK -> $(readlink -f "$INSTALL_FILE")"
    else
        echo "Type: REGULAR FILE (not a symlink — this means --symlink-install did NOT take effect for this file)"
    fi
    echo "Top-level key:"
    head -1 "$INSTALL_FILE"
else
    echo "MISSING: $INSTALL_FILE  <-- if missing, package needs (re)building"
fi

echo ""
echo "--- 3. COMPARISON ---"
if [ -f "$SRC_FILE" ] && [ -f "$INSTALL_FILE" ]; then
    if diff -q "$SRC_FILE" "$INSTALL_FILE" > /dev/null 2>&1; then
        echo "MATCH: source and installed files are identical."
    else
        echo "MISMATCH: source and installed files DIFFER."
        echo ""
        echo "Diff:"
        diff "$SRC_FILE" "$INSTALL_FILE" || true
        echo ""
        echo ">>> FIX: run the following to force a real rebuild:"
        echo "    rm -rf build/butler_navigation install/butler_navigation"
        echo "    colcon build --symlink-install --packages-select butler_navigation"
    fi
fi

echo ""
echo "--- 4. WHICH FILE WOULD THE LAUNCH ACTUALLY USE RIGHT NOW ---"
echo "(this mirrors get_package_share_directory('butler_navigation') used in the launch file)"
if command -v ros2 &> /dev/null; then
    python3 -c "
try:
    from ament_index_python.packages import get_package_share_directory
    import os
    pkg = get_package_share_directory('butler_navigation')
    path = os.path.join(pkg, 'config', 'locations.yaml')
    print('Resolved path:', path)
    if os.path.exists(path):
        with open(path) as f:
            print('First line:', f.readline().strip())
    else:
        print('FILE DOES NOT EXIST AT THIS PATH')
except Exception as e:
    print('Could not resolve (did you source install/setup.bash?):', e)
"
else
    echo "ros2 not on PATH in this shell — source install/setup.bash first."
fi

echo ""
echo "=========================================="
echo " END DIAGNOSTIC"
echo "=========================================="
