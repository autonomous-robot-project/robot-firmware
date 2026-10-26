#pragma once

// Constants of the control: no hardware, only the standard library, so the control classes that
// use them also build and run on the host for the unit tests (robot-firmware/test/host).
// Symbols of the verifications; plain constexpr, as in config.h.

#include <chrono>
#include <cstdint>
#include <numbers>

// --- Timing (V-05, V-06) ---
constexpr std::chrono::milliseconds T_C{10};  // period of the control task
constexpr float T_C_SECONDS = std::chrono::duration<float>(T_C).count();

// --- Duty limit (ADR 023) ---
constexpr float DUTY_MAX = 0.48f;  // the stalled wheel gets at most the motor rated voltage

// --- Wheel and encoder (V-02) ---
constexpr float WHEEL_DIAMETER_M = 0.065f;  // D
constexpr int ENCODER_SLOTS = 20;           // N, rising edges per turn
// s (m): wheel travel between two rising edges, 10.2 mm
constexpr float WHEEL_TRAVEL_PER_EDGE = std::numbers::pi_v<float> * WHEEL_DIAMETER_M /
                                        ENCODER_SLOTS;

// --- Reference: acceleration limits (ADR 023) and inverse kinematics; provisional values ---
constexpr float A_ACCEL = 0.25f;  // m/s², limited by the current and the gearbox (robot-docs#27)
constexpr float A_DEC = 0.5f;     // m/s², limited by slip; steeper than A_ACCEL (robot-docs#27)
constexpr float TRACK_WIDTH = 0.15f;  // b (m), between the wheels (robot-docs#25)

// --- Wheel PI: gains from the motor model and the settling time ---
// Motor as a first-order plant, speed / duty = K / (tau·s + 1), from characterize.py.
// K, (m/s) per unit of duty; provisional: the TT motor makes ~0.68 m/s at 6 V, duty 0.48.
constexpr float MOTOR_GAIN = 1.4f;
constexpr float MOTOR_TIME_CONSTANT = 0.1f;  // tau (s); provisional
// Settling time (2 %) of the wheel speed, a design choice: at least ~0.4 s, so the closed loop is
// slower than the edges at 0.2 m/s (one every ~51 ms), and at most ~0.95 s, so a stall is detected
// within the 2 s of V-02.
constexpr float WHEEL_SETTLING_TIME = 0.5f;  // t_s (s)
// The zero of the PI cancels the motor pole: first-order closed loop with time constant t_s / 4,
// no overshoot.
constexpr float KP = 4.0f * MOTOR_TIME_CONSTANT / (MOTOR_GAIN * WHEEL_SETTLING_TIME);  // per m/s
constexpr float KI = KP / MOTOR_TIME_CONSTANT;                                         // per m

// --- Fault detection (V-02) ---
constexpr float E_F = 0.5f;                        // e_f: speed error that counts as a fault
constexpr float T_F = 2.0f * WHEEL_SETTLING_TIME;  // T_f (s): persistence of the error
constexpr float V_MIN = WHEEL_TRAVEL_PER_EDGE / ((1.0f - E_F) * T_F);  // v_min (m/s)
// No edge for T_VEL: speed zero (= s / v_min).
constexpr std::chrono::microseconds T_VEL{static_cast<int64_t>((1.0f - E_F) * T_F * 1e6f)};
