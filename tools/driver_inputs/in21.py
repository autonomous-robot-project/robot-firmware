"""robot-docs#21 (nota 0009): captures the driver inputs and reports every drive state (IN1 != IN2).

Runs on the RPi with the logic analyzer (fx2lafw clone, sigrok-cli). The analyzer pulls each input
up through ~66 kOhm: readings of 1 while the ESP32 does not drive the pin (reset, boot, ESP32 off)
come from the analyzer, readings of 0 are valid. With the motor supply on, never remove the probe
of only one input of a driver: the driver would see IN1 != IN2 and turn the motor.

Usage: python3 in21.py <label> <seconds>
Channels: D0/D1 = left IN1/IN2 (GPIO 4/5), D2/D3 = right IN1/IN2 (GPIO 2/42),
D4 = RST (EN), D5 = watchdog test pin (GPIO 11). 2 MHz sampling.
"""

import subprocess
import sys

import numpy as np

SAMPLE_RATE = 2_000_000
STATE_NAMES = {0b00: 'free', 0b11: 'brake', 0b01: 'FORWARD', 0b10: 'BACKWARD'}  # bit0 = IN1


def capture(label, seconds):
    samples = int(seconds * SAMPLE_RATE)
    subprocess.run(
        ['sigrok-cli', '-d', 'fx2lafw', '--config', f'samplerate={SAMPLE_RATE}',
         '--samples', str(samples), '-o', f'{label}.sr'],
        check=True,
    )
    raw = subprocess.run(
        ['sigrok-cli', '-i', f'{label}.sr', '-O', 'binary'], check=True, capture_output=True
    ).stdout
    return np.frombuffer(raw, dtype=np.uint8)


def segments(values):
    """(start index, length, value) of each run of equal values."""
    change = np.flatnonzero(np.diff(values)) + 1
    starts = np.concatenate(([0], change))
    lengths = np.diff(np.concatenate((starts, [len(values)])))
    return [(int(s), int(n), int(values[s])) for s, n in zip(starts, lengths)]


def ms(samples):
    return samples * 1000.0 / SAMPLE_RATE


def report_driver(name, data, in1_bit, in2_bit):
    state = ((data >> in1_bit) & 1) | (((data >> in2_bit) & 1) << 1)
    runs = segments(state)
    drive = [r for r in runs if STATE_NAMES[r[2]] in ('FORWARD', 'BACKWARD')]
    print(f'{name}: {len(runs)} segments; drive segments: {len(drive)}', end='')
    if drive:
        longest = max(drive, key=lambda r: r[1])
        print(f', longest {ms(longest[1]) * 1000:.1f} us at {ms(longest[0]):.3f} ms', end='')
    print()
    shown = runs if len(runs) <= 30 else runs[:15] + runs[-15:]
    for start, length, value in shown:
        print(f'    {ms(start):10.3f} ms  {ms(length):10.3f} ms  {STATE_NAMES[value]}')
    if len(runs) > 30:
        print(f'    ... ({len(runs) - 30} segments omitted)')


def report_edges(name, data, bit):
    line = (data >> bit) & 1
    runs = segments(line)
    edges = ', '.join(f'{ms(s):.3f} ms -> {v}' for s, _, v in runs[1:][:20])
    print(f'{name}: starts {runs[0][2]}; edges: {edges or "none"}')


def main():
    label, seconds = sys.argv[1], float(sys.argv[2])
    data = capture(label, seconds)
    print(f'{label}: {len(data)} samples ({ms(len(data)) / 1000:.1f} s)')
    report_edges('RST', data, 4)
    report_edges('GPIO 11', data, 5)
    report_driver('left driver (D0/D1)', data, 0, 1)
    report_driver('right driver (D2/D3)', data, 2, 3)


if __name__ == '__main__':
    main()
