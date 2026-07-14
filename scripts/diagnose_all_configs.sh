#!/usr/bin/env bash
# diagnose_all_configs.sh
# =========================
# Generalised version of diagnose_locations.sh. Walks every file under
# src/*/config/ and src/*/launch/, finds its corresponding installed
# copy under install/*/share/*/..., and reports:
#   - whether the installed copy exists
#   - whether it's a symlink (correct, --symlink-install worked) or a
#     stale regular-file copy
#   - whether its content matches the source file
#
# This exists because the exact same bug class (source fixed, installed
# copy stale) has now hit locations.yaml AND nav2_params.yaml in this
# project. Run this BEFORE every relaunch when debugging something that
# "should already be fixed" to rule out staleness in one shot, instead
# of re-discovering it through a full Gazebo/Nav2 boot cycle.
#
# Usage:
#   cd ~/butler_robot_ws
#   bash scripts/diagnose_all_configs.sh
#
# Exit code 0 = everything matches. Exit code 1 = at least one mismatch
# or missing installed file was found (printed in detail above).

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT" || exit 1

echo "=========================================================="
echo " WORKSPACE-WIDE SRC vs INSTALL DIAGNOSTIC"
echo " Workspace: $WS_ROOT"
echo "=========================================================="

PROBLEMS=0
CHECKED=0

# Find every package directory under src/
for pkg_dir in src/*/; do
    pkg_name="$(basename "$pkg_dir")"

    # Check both config/ and launch/ subdirectories
    for subdir in config launch; do
        src_subdir="${pkg_dir}${subdir}"
        [ -d "$src_subdir" ] || continue

        for src_file in "$src_subdir"/*; do
            [ -f "$src_file" ] || continue
            CHECKED=$((CHECKED + 1))

            rel_path="${src_file#"$pkg_dir"}"   # e.g. "config/locations.yaml"
            install_file="install/${pkg_name}/share/${pkg_name}/${rel_path}"

            echo ""
            echo "--- ${pkg_name}/${rel_path} ---"
            echo "  source:    $src_file"

            if [ ! -e "$install_file" ]; then
                echo "  installed: MISSING ($install_file)"
                echo "  STATUS:    NOT BUILT YET — run colcon build for $pkg_name"
                PROBLEMS=$((PROBLEMS + 1))
                continue
            fi

            echo "  installed: $install_file"

            if [ -L "$install_file" ]; then
                target="$(readlink -f "$install_file")"
                echo "  type:      symlink -> $target"
            else
                echo "  type:      REGULAR FILE (not symlinked — possible staleness risk)"
            fi

            if diff -q "$src_file" "$install_file" > /dev/null 2>&1; then
                echo "  STATUS:    MATCH"
            else
                echo "  STATUS:    *** MISMATCH — installed copy is STALE ***"
                echo "  ---- diff (source vs installed) ----"
                diff "$src_file" "$install_file" | head -10
                echo "  ---- end diff ----"
                PROBLEMS=$((PROBLEMS + 1))
            fi
        done
    done
done

echo ""
echo "=========================================================="
echo " SUMMARY: checked $CHECKED file(s), found $PROBLEMS problem(s)"
echo "=========================================================="

if [ "$PROBLEMS" -gt 0 ]; then
    echo ""
    echo "To fix ALL stale packages found above in one shot:"
    echo ""
    echo "  cd ~/butler_robot_ws"
    echo "  rm -rf build install log"
    echo "  colcon build --symlink-install"
    echo "  source install/setup.bash"
    echo ""
    echo "Then re-run this script to confirm everything now says MATCH."
    exit 1
else
    echo ""
    echo "All checked files match between src/ and install/. No staleness detected."
    exit 0
fi
