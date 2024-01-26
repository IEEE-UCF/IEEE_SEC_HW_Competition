#!/bin/bash

set -e

source /opt/ros/humble/install/setup.bash

pip3 install smbus
pip install pyserial

echo "Provided arguments: $@"

exec $@
