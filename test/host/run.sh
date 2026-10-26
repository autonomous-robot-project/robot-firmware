#!/bin/sh
# Builds and runs the unit tests of the control classes on the host: plain g++, no ESP-IDF.
set -e
cd "$(dirname "$0")"
CONTROLLER=../../components/controller
BUILD=$(mktemp -d)
g++ -std=c++20 -Wall -Wextra -Werror \
    -I "$CONTROLLER/include" -I "$CONTROLLER/private_include" \
    test_wheel_pi.cpp "$CONTROLLER/src/control/wheel_pi.cpp" \
    -o "$BUILD/test_wheel_pi"
"$BUILD/test_wheel_pi"
