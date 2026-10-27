#pragma once

// Constants of the control: no hardware, only the standard library, so the control classes that
// use them also build and run on the host for the unit tests (robot-firmware/test/host).
// Symbols of the verifications; plain constexpr, as in config.h.

#include <algorithm>
#include <chrono>
#include <cmath>
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
// K, (m/s) per unit of duty, and tau from the encoder AO run, wheels suspended (robot-docs nota
// 0010): K from 1.0 to 1.45 and tau from 30 to 57 ms over the wheels and directions; the medians.
constexpr float MOTOR_GAIN = 1.3f;
constexpr float MOTOR_TIME_CONSTANT = 0.04f;  // tau (s)
// Settling time (2 %) of the wheel speed, a design choice: at least ~0.4 s, so the closed loop is
// slower than the edges at 0.2 m/s (one every ~51 ms), and at most ~0.95 s, so a stall is detected
// within the 2 s of V-02.
constexpr float WHEEL_SETTLING_TIME = 0.5f;  // t_s (s)

// Discrete design at T_C. With the zero-order hold of the PWM and the one-cycle delay of the
// measurement, the motor is G(z) = b / (z (z - a)), with a = e^(-T/tau) and b = K (1 - a).
// The PI zero cancels the motor pole a, so the closed loop is z^2 - z + Kp b = 0. Its dominant
// pole p = e^(-T / (t_s / 4)) gives Kp b = p (1 - p); the poles are real (no overshoot) while
// Kp b <= 1/4. std::exp is not constexpr in C++20, so these are const.
const float MOTOR_POLE = std::exp(-T_C_SECONDS / MOTOR_TIME_CONSTANT);                 // a
const float MOTOR_STEP_GAIN = MOTOR_GAIN * (1.0f - MOTOR_POLE);                        // b
const float CLOSED_LOOP_POLE = std::exp(-T_C_SECONDS / (WHEEL_SETTLING_TIME / 4.0f));  // p
const float KP = CLOSED_LOOP_POLE * (1.0f - CLOSED_LOOP_POLE) / MOTOR_STEP_GAIN;       // per m/s
// u = Kp e + Ki T sum(e) has its zero at Kp / (Kp + Ki T): at a when Ki T = Kp (1 - a) / a.
const float KI = KP * (1.0f - MOTOR_POLE) / (MOTOR_POLE * T_C_SECONDS);  // per m

// --- Fault detection (V-02) and lower limit of the wheel speed ---
constexpr float E_F = 0.5f;                        // e_f: speed error that counts as a fault
constexpr float T_F = 2.0f * WHEEL_SETTLING_TIME;  // T_f (s): persistence of the error

// Detection limit: a stalled wheel must read below (1 - e_f) v within T_f, so
// v >= s / ((1 - e_f) T_f), about 0.020 m/s.
constexpr float V_MIN_DETECTION = WHEEL_TRAVEL_PER_EDGE / ((1.0f - E_F) * T_F);
// Control limit: below about 0.07 m/s the edges are so sparse that the loop falls into a limit
// cycle (host test with the encoder model; provisional, to confirm with the motor).
constexpr float V_MIN_CONTROL = 0.07f;
// v_min (m/s): the larger of the two. Below it the PI brakes (ADR 030).
constexpr float V_MIN = std::max(V_MIN_DETECTION, V_MIN_CONTROL);

// No edge for T_VEL: speed zero. It comes from the detection limit, (1 - e_f) T_f: with the
// control limit it would equal the edge interval at v_min, and the estimate would drop to zero
// between two edges.
constexpr std::chrono::microseconds T_VEL{
    static_cast<int64_t>(WHEEL_TRAVEL_PER_EDGE / V_MIN_DETECTION * 1e6f)
};
