#!/bin/bash

<<<<<<< Updated upstream
set -e

source /opt/ros/humble/install/setup.bash

pip3 install smbus
pip install pyserial

=======
>>>>>>> Stashed changes
echo "Provided arguments: $@"

exec $@
