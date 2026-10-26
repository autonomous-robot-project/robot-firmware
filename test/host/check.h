#pragma once

#include <cstdio>

// Minimal checks for the host tests: each one prints PASS or FAIL; main returns the verdict.
inline int failures = 0;

inline void check(bool condition, const char *description)
{
    std::printf("%s: %s\n", condition ? "PASS" : "FAIL", description);
    if (!condition)
    {
        failures++;
    }
}

inline int verdict()
{
    std::printf("%s\n", failures == 0 ? "all passed" : "FAILED");
    return failures == 0 ? 0 : 1;
}
