#!/bin/sh
modprobe can
modprobe can-dev
modprobe mttcan
ip link set can0 up type can bitrate 500000
. /root/tracer_driver_ws/install/setup.sh
exec "$@"
