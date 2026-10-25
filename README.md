# robot-firmware

Embedded code for motor actuation, sensor reading, communication with the robot's main system and security logic.

## Static analysis

clang-tidy with `.clang-tidy`, using the esp-clang toolchain (`idf_tools.py install esp-clang`, `pip install pyclang`) in a separate build directory, so the GCC build stays untouched:

```
IDF_TOOLCHAIN=clang idf.py -B build-clang clang-check --run-clang-tidy-options "-config-file=$PWD/.clang-tidy"
```

The report goes to `warnings.txt`. `IDF_TOOLCHAIN` goes in the environment: `clang-check` reconfigures the build on its own and ignores a `-D` option.
