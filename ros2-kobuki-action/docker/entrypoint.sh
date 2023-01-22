#!/bin/bash                                                                     
set -e
source /opt/ros/galactic/setup.bash
source /kobuki_actions_ws/install/setup.bash

exec "$@"