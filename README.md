# robot-firmware

Embedded code for motor actuation, sensor reading, communication with the robot's main system and security logic.

## Static analysis

clang-tidy with `.clang-tidy`, using the esp-clang toolchain (`idf_tools.py install esp-clang`, `pip install pyclang`) in a separate build directory, so the GCC build stays untouched:

```
IDF_TOOLCHAIN=clang idf.py -B build-clang clang-check --run-clang-tidy-options "-config-file=$PWD/.clang-tidy"
```

The report goes to `warnings.txt`. `IDF_TOOLCHAIN` goes in the environment: `clang-check` reconfigures the build on its own and ignores a `-D` option.

## Bench tools

`tools/motor_characterization`: open-loop characterization of the motors, with the wheels suspended. A separate ESP-IDF app that reuses the `controller` component (no link, no micro-ROS). Build it in its folder with `idf.py build`, flash the app at `0x10000` as the robot firmware, and run on the RPi `python3 characterize.py record <USB port> <file.csv>`: the script starts the run, records the CSV and prints the dead zone, the steady speed and the time constant of each step, and the line speed = gain · (|duty| − dead zone). Flash the robot firmware back afterwards.

The brake after each step is the full brake, which stresses the motor and the gearbox mechanically and thermally. The steps go from the lowest duty up: watch the first brakes, at low speed, and cut the motor supply if the gearbox shows trouble (ADR 023).
