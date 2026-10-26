"""
Open-loop characterization of the motors, with the wheels suspended (bench tool, slice 2).

Records the CSV that the motor_characterization app prints on the USB console and reports, for
each wheel:
- the duty at which the wheel starts and stops on the slow ramp;
- for each duty step, the steady speed and the time constant tau of a first-order response;
- for the brake after each step, the braking time constant (TWIST(0, 0) is the full brake);
- the line speed = gain * (|duty| - dead zone) fitted to the steps.

Each step is read from the travelled distance (the edges times s), not from s / dt: after a
step, the distance of a first-order response tends to the line v * (t - tau), so the slope of the
line fitted to the end of the step is the steady speed and its time intercept is tau. The speed
samples only exist at the edges and would delay the 63 % instant. Likewise, a first-order stop
from the speed v covers v * tau_brake, so tau_brake is the distance after the step over v.

Usage, on the RPi with the app flashed:
    python3 characterize.py record <usb port> <file.csv>   # sends the start line, then analyzes
    python3 characterize.py analyze <file.csv>
"""

from dataclasses import dataclass
from itertools import groupby
import math
from statistics import mean
import sys

WHEEL_TRAVEL_PER_EDGE = math.pi * 0.065 / 20  # s (m): D = 65 mm, N = 20 (V-02)
RECORD_TIMEOUT_S = 150.0
MIN_STEP_S = 1.0  # a constant non-zero duty at least this long is a step
STEADY_WINDOW_S = 1.0  # the end of the step where the response is steady
BRAKE_WINDOW_S = 1.9  # after the step, within the 2 s of brake of the app
MIN_FIT_EDGES = 3
WHEELS = ('L', 'R')


@dataclass
class DutyChange:
    time: float
    duty: float


@dataclass
class Line:
    """Least-squares line y = slope * x + intercept."""

    slope: float
    intercept: float

    @classmethod
    def fit(cls, xs, ys):
        x_mean, y_mean = mean(xs), mean(ys)
        covariance = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys, strict=True))
        variance = sum((x - x_mean) ** 2 for x in xs)
        slope = covariance / variance
        return cls(slope, y_mean - slope * x_mean)


class Recording:
    """The duty changes and the encoder edges of one run, with times in seconds."""

    def __init__(self, lines):
        self.duty_changes = []
        self.edges = {wheel: [] for wheel in WHEELS}
        for line in lines:
            fields = line.strip().split(',')
            if fields[0] == 'duty':
                self.duty_changes.append(DutyChange(int(fields[1]) / 1e6, float(fields[2])))
            elif fields[0] == 'edge':
                self.edges[fields[2]].append(int(fields[1]) / 1e6)

    @classmethod
    def from_file(cls, path):
        with open(path) as file:
            return cls(file)

    def duty_at(self, time):
        applied = [change.duty for change in self.duty_changes if change.time <= time]
        return applied[-1] if applied else 0.0


@dataclass
class Step:
    """A constant non-zero duty held for at least MIN_STEP_S."""

    start: float
    end: float
    duty: float

    @staticmethod
    def find_all(recording):
        steps = []
        for duty, group in groupby(recording.duty_changes, key=lambda change: change.duty):
            changes = list(group)
            start, end = changes[0].time, changes[-1].time
            if duty != 0.0 and end - start >= MIN_STEP_S:
                steps.append(Step(start, end, duty))
        return steps


@dataclass
class StepResponse:
    duty: float
    steady_speed: float  # m/s; 0 if too few edges at the end of the step
    time_constant: float  # s; nan if too few edges
    braking_time_constant: float  # s; nan if too few edges

    @classmethod
    def of(cls, step, edges):
        edges_in_step = [edge for edge in edges if step.start < edge <= step.end]
        distances = cls._distances(len(edges_in_step))
        steady = [
            (time, distance)
            for time, distance in zip(edges_in_step, distances, strict=True)
            if time > step.end - STEADY_WINDOW_S
        ]
        if len(steady) < MIN_FIT_EDGES:
            return cls(step.duty, 0.0, math.nan, math.nan)

        line = Line.fit([time for time, _ in steady], [distance for _, distance in steady])
        time_at_zero_distance = -line.intercept / line.slope

        edges_while_braking = [
            edge for edge in edges if step.end < edge <= step.end + BRAKE_WINDOW_S
        ]
        braking_distance = len(edges_while_braking) * WHEEL_TRAVEL_PER_EDGE

        return cls(
            duty=step.duty,
            steady_speed=line.slope,
            time_constant=time_at_zero_distance - step.start,
            braking_time_constant=braking_distance / line.slope,
        )

    @staticmethod
    def _distances(edge_count):
        # The wheel starts at an unknown point of a slot: on average, half a slot to the 1st edge.
        return [(index + 0.5) * WHEEL_TRAVEL_PER_EDGE for index in range(edge_count)]


class StaticMap:
    """The line speed = gain * (|duty| - dead zone), fitted to the steady speed of the steps."""

    def __init__(self, responses):
        moving = [response for response in responses if response.steady_speed > 0.0]
        line = Line.fit(
            [abs(response.duty) for response in moving],
            [response.steady_speed for response in moving],
        )
        self.gain = line.slope  # (m/s) per unit of duty
        self.dead_zone = -line.intercept / line.slope  # duty


class DeadZone:
    """Duty at the first edge of the ramp up and at the last edge of the ramp down."""

    def __init__(self, recording, edges, ramp_end):
        peak = max(recording.duty_changes, key=lambda change: change.duty).time
        rising = [edge for edge in edges if edge <= peak]
        falling = [edge for edge in edges if peak < edge < ramp_end]
        self.start_duty = recording.duty_at(rising[0]) if rising else math.nan
        self.stop_duty = recording.duty_at(falling[-1]) if falling else math.nan


class Report:
    def __init__(self, recording):
        self.recording = recording

    def print(self):
        steps = Step.find_all(self.recording)
        ramp_end = steps[0].start if steps else math.inf
        for wheel in WHEELS:
            edges = self.recording.edges[wheel]
            dead_zone = DeadZone(self.recording, edges, ramp_end)
            print(f'Wheel {wheel}: {len(edges)} edges')
            print(
                f'  dead zone: starts at duty {dead_zone.start_duty:.3f}, '
                f'stops at duty {dead_zone.stop_duty:.3f}'
            )
            responses = [StepResponse.of(step, edges) for step in steps]
            print('   duty  speed (m/s)  tau (ms)  tau_brake (ms)')
            for response in responses:
                print(
                    f'  {response.duty:+.2f}  {response.steady_speed:11.3f}  '
                    f'{response.time_constant * 1e3:8.0f}  '
                    f'{response.braking_time_constant * 1e3:14.0f}'
                )
            static_map = StaticMap(responses)
            print(
                f'  fit: speed = {static_map.gain:.3f} m/s * (|duty| - {static_map.dead_zone:.3f})'
            )


def record(port, path):
    import serial  # only the recording needs the serial port

    with serial.Serial(port, timeout=RECORD_TIMEOUT_S) as console, open(path, 'w') as file:
        console.reset_input_buffer()
        console.write(b'\n')
        while True:
            line = console.readline().decode(errors='replace')
            if not line:
                raise TimeoutError(f'no output for {RECORD_TIMEOUT_S} s')
            file.write(line)
            if line.strip() == 'done':
                return


def main():
    command, *arguments = sys.argv[1:]
    if command == 'record':
        port, path = arguments
        record(port, path)
    elif command == 'analyze':
        (path,) = arguments
    else:
        sys.exit(__doc__)
    Report(Recording.from_file(path)).print()


if __name__ == '__main__':
    main()
