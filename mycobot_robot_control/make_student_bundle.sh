#!/usr/bin/env bash
# Create a clean tarball to give to students.
#
# The bundle excludes local build products, logs, reports, and Python cache
# files. Students rebuild the ROS workspace on their own laptops.
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_NAME="$(basename "$WORKSPACE_DIR")"
PARENT_DIR="$(dirname "$WORKSPACE_DIR")"
OUT="${1:-/tmp/mycobot_robot_control_student.tar.gz}"

mkdir -p "$(dirname "$OUT")"

tar -czf "$OUT" -C "$PARENT_DIR" \
  --exclude="$WORKSPACE_NAME/build" \
  --exclude="$WORKSPACE_NAME/install" \
  --exclude="$WORKSPACE_NAME/log" \
  --exclude="$WORKSPACE_NAME/reports" \
  --exclude="$WORKSPACE_NAME/.git" \
  --exclude="$WORKSPACE_NAME/.agents" \
  --exclude="$WORKSPACE_NAME/.codex" \
  --exclude="*/__pycache__" \
  --exclude="*.pyc" \
  --exclude="*.pyo" \
  "$WORKSPACE_NAME"

echo "student bundle: $OUT"
echo
echo "Student install:"
echo "  cd ~"
echo "  tar xzf $OUT"
echo "  cd $WORKSPACE_NAME"
echo "  bash laptop_bootstrap.sh"

if [ ! -s "$WORKSPACE_DIR/provision/mycobot-ros2-1.1.0.tar" ]; then
  echo
  echo "Note: provision/mycobot-ros2-1.1.0.tar is not present."
  echo "Fresh robots without the comms image will need that tarball copied into provision/."
fi
