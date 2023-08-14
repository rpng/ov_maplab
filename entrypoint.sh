#!/bin/bash
set -e

# setup ros environment
# ensure you have already built the workspace!
source "/catkin_ws/devel/setup.bash" --
exec "$@"
