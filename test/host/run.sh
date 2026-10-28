#!/bin/sh
# Builds and runs the unit tests of the control classes on the host: plain g++, no ESP-IDF.
set -e
cd "$(dirname "$0")"
CONTROLLER=../../components/controller
BUILD=$(mktemp -d)
build() {
    g++ -std=c++20 -Wall -Wextra -Werror \
        -I "$CONTROLLER/include" -I "$CONTROLLER/private_include" \
        "$@"
}
MODEL="$CONTROLLER/src/control/wheel_pi.cpp $CONTROLLER/src/sensing/wheel_speed.cpp"

build test_wheel_pi.cpp $MODEL -o "$BUILD/test_wheel_pi"
build test_wheel_tracking_monitor.cpp $MODEL \
    "$CONTROLLER/src/control/acceleration_limiter.cpp" \
    "$CONTROLLER/src/supervision/wheel_tracking_monitor.cpp" \
    -o "$BUILD/test_wheel_tracking_monitor"
build test_wheel_odometry.cpp "$CONTROLLER/src/sensing/wheel_odometry.cpp" \
    -o "$BUILD/test_wheel_odometry"
build test_behaviors.cpp "$CONTROLLER/src/control/turn_to.cpp" "$CONTROLLER/src/control/go_to.cpp" \
    -o "$BUILD/test_behaviors"
build test_deadline_meter.cpp "$CONTROLLER/src/instrumentation/deadline_meter.cpp" \
    -o "$BUILD/test_deadline_meter"

echo "== WheelPI"
"$BUILD/test_wheel_pi"
echo "== WheelTrackingMonitor"
"$BUILD/test_wheel_tracking_monitor"
echo "== WheelOdometry"
"$BUILD/test_wheel_odometry"
echo "== TURN_TO and GO_TO"
"$BUILD/test_behaviors"
echo "== DeadlineMeter"
"$BUILD/test_deadline_meter"
