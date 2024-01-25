#!/bin/bash

set -e

source /opt/ros/humble/install/setup.bash

echo "Provided arguments: $@"

exec $@
