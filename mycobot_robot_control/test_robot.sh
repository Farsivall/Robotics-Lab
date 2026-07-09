#!/usr/bin/env bash
set -o pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/src/mycobot_robot_control/scripts/test_robot.sh" "$@"
