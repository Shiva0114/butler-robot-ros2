#!/usr/bin/env python3
"""
check_param_yaml_node_names.py
================================
Guards against a recurring bug class: a ROS2 parameter YAML file's
top-level key must exactly match the node name it will be loaded into
(the name set via super().__init__(...) in code, or overridden by the
launch file's name= argument). If they don't match, ROS2 silently loads
zero parameters with no error — exactly what happened three times in
this project's history (location_server -> nav_bridge -> robot_state_machine
renames, where the YAML key was never updated to match).

Usage:
    python3 scripts/check_param_yaml_node_names.py

Run this any time you rename a node or edit a parameter YAML file.
It exits non-zero and prints a clear error if any mismatch is found.
"""
import re
import sys
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parent.parent
SRC = WS_ROOT / "src"


def find_node_names_in_launch_files():
    """
    Scan all launch/*.py files for Node(...) blocks and extract
    (executable, name) pairs. Returns dict: executable -> launch_name
    """
    mapping = {}
    for launch_file in SRC.glob("*/launch/*.py"):
        text = launch_file.read_text()
        # crude but effective: find each Node( ... ) block via brace matching
        for match in re.finditer(r"Node\(([^)]*?)\)", text, re.DOTALL):
            block = match.group(1)
            exe_match = re.search(r'executable\s*=\s*"([^"]+)"', block)
            name_match = re.search(r'name\s*=\s*"([^"]+)"', block)
            if exe_match and name_match:
                mapping[exe_match.group(1)] = name_match.group(1)
    return mapping


def find_default_node_name(executable: str) -> str:
    """
    For a given executable (e.g. 'state_machine_node.py'), find the
    corresponding Python source file and extract the default node name
    from super().__init__("...").
    """
    for py_file in SRC.glob(f"*/*/{executable}"):
        text = py_file.read_text()
        match = re.search(r'super\(\)\.__init__\(\s*"([^"]+)"', text)
        if match:
            return match.group(1)
    return None


def find_yaml_files_passed_to_node(executable: str):
    """
    Find launch files that pass a parameter YAML file to this executable,
    and return the list of (launch_file, yaml_path_expression) found.
    This is approximate — it looks for any *.yaml reference within the
    same Node(...) block as the executable.
    """
    results = []
    for launch_file in SRC.glob("*/launch/*.py"):
        text = launch_file.read_text()
        for match in re.finditer(r"Node\(([^)]*?)\)", text, re.DOTALL):
            block = match.group(1)
            if f'executable="{executable}"' not in block and f"executable='{executable}'" not in block:
                continue
            for yaml_var in re.findall(r"\b(\w*_cfg|\w*_params|\w*yaml\w*)\b", block):
                results.append((launch_file, yaml_var))
    return results


def get_yaml_top_level_keys(yaml_path: Path):
    text = yaml_path.read_text()
    keys = []
    for line in text.splitlines():
        if line and not line.startswith((" ", "\t", "#")) and line.rstrip().endswith(":"):
            keys.append(line.rstrip().rstrip(":"))
    return keys


def main():
    errors = []

    print("Scanning launch files for Node(...) executable -> name mappings...\n")
    exe_to_launch_name = find_node_names_in_launch_files()

    for exe, launch_name in exe_to_launch_name.items():
        default_name = find_default_node_name(exe)
        print(f"  {exe:30s} launch name='{launch_name}'  code default='{default_name}'")

    print("\nChecking all parameter YAML files under config/ directories...\n")
    for yaml_path in SRC.glob("*/config/*.yaml"):
        top_keys = get_yaml_top_level_keys(yaml_path)
        rel = yaml_path.relative_to(WS_ROOT)
        print(f"  {rel}")
        print(f"    top-level keys: {top_keys}")

        # Check if any node name in our mapping matches one of the top-level keys
        matched = any(k in exe_to_launch_name.values() for k in top_keys)
        if not matched and top_keys:
            # Only warn if this looks like a node-params file (not e.g. nav2 multi-node files
            # which legitimately have many top-level keys for different nodes)
            if len(top_keys) <= 2:
                errors.append(
                    f"    WARNING: top-level key(s) {top_keys} in {rel} do not match "
                    f"any known launched node name {sorted(set(exe_to_launch_name.values()))}. "
                    f"If this file is meant for one of those nodes, the params will load empty."
                )

    if errors:
        print("\n" + "=" * 70)
        print("POTENTIAL MISMATCHES FOUND:")
        print("=" * 70)
        for e in errors:
            print(e)
        print("\nIf any of these are real bugs, fix the YAML top-level key to exactly")
        print("match the node's name (the 'name=' argument in the launch file, or the")
        print("string passed to super().__init__(...) if no override is given).")
        sys.exit(1)
    else:
        print("\nNo obvious node-name / YAML-key mismatches detected.")
        sys.exit(0)


if __name__ == "__main__":
    main()
