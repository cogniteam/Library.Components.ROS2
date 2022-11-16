#!/bin/bash
set -e

source /opt/ros/eloquent/setup.bash
source /colcon_ws/install/setup.bash

exec "$@"
