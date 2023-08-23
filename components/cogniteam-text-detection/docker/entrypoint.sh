#!/bin/bash                                                                     
set -e
source /opt/ros/foxy/setup.bash
source /text_detection_ws/install/setup.bash

exec "$@"