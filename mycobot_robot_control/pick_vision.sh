#!/usr/bin/env bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/src/mycobot_robot_control/scripts/pick_vision.sh" "$@"
