"""
Rangings of the front sensor (bench tool, V-03): the data of T_S and of the status table of ADR 021
(robot-docs nota 0015).

Records the CSV that the front_range app prints on the USB console, one scenario per file, and
reports:
- the intervals between consecutive rangings;
- for each device status: count, distance, return signal rate per SPAD (the quantity of the range
  ignore threshold of the ST API) and the longest gap between rangings of the other
  statuses, which is the time without a classifiable reading if that status alone were not
  classifiable;
- with the statuses that the status table leaves unclassifiable, the longest gap between the
  classifiable rangings, from which T_S is set (ADR 021).

Usage, on the RPi with the app flashed:
    python3 front_range.py record <usb port> <file.csv> <seconds>
    python3 front_range.py analyze <file.csv> [unclassifiable statuses, e.g. 1,2]
Criticality: T1, evidence (robot-docs codigo.md).
"""

import re
import sys
import time
from dataclasses import dataclass
from itertools import pairwise
from statistics import median, quantiles

READ_TIMEOUT_S = 1.0  # a silent console still lets the recording end on time
LOG_PREFIXES = ('W (', 'E (')  # warnings and errors of the sensor task (ESP-IDF log)
COLOR_CODES = re.compile(r'\x1b\[[0-9;]*m')  # the ESP-IDF log colors its lines


@dataclass
class Ranging:
    time: float  # s, measured_at of the sensor task
    distance_mm: int
    status: int
    signal_per_spad_mcps: float  # signal rate (9.7) over effective SPADs (8.8)


class Recording:
    """The rangings and the log lines of the sensor task of one scenario."""

    def __init__(self, lines):
        self.rangings = []
        self.log_lines = []
        for raw_line in lines:
            line = COLOR_CODES.sub('', raw_line)
            fields = line.strip().split(',')
            if fields[0] == 'range':
                self.rangings.append(Ranging(*parse_ranging(fields[1:])))
            elif line.startswith(LOG_PREFIXES):
                self.log_lines.append(line.strip())

    @classmethod
    def from_file(cls, path):
        with open(path) as file:
            return cls(file)

    def statuses(self):
        return sorted({ranging.status for ranging in self.rangings})

    def longest_gap(self, unclassifiable):
        """Longest time between consecutive rangings whose status is not in `unclassifiable`."""
        times = [ranging.time for ranging in self.rangings if ranging.status not in unclassifiable]
        return max((later - earlier for earlier, later in pairwise(times)), default=None)


def parse_ranging(fields):
    t_us, distance_mm, status, signal_q9_7, spads_q8_8 = (int(field) for field in fields)
    spads = spads_q8_8 / 256
    signal_per_spad = (signal_q9_7 / 128) / spads if spads else 0.0
    return t_us / 1e6, distance_mm, status, signal_per_spad


def milliseconds(seconds):
    return f'{seconds * 1e3:.1f} ms' if seconds is not None else 'no gap (under 2 rangings)'


def print_intervals(recording):
    times = [ranging.time for ranging in recording.rangings]
    intervals = [later - earlier for earlier, later in pairwise(times)]
    if len(intervals) < 2:
        print('intervals: fewer than 3 rangings')
        return
    # Inclusive: the percentile stays within the measured intervals, never above the maximum.
    p99 = quantiles(intervals, n=100, method='inclusive')[98]
    print(
        f'intervals: min {milliseconds(min(intervals))}, median {milliseconds(median(intervals))}, '
        f'p99 {milliseconds(p99)}, max {milliseconds(max(intervals))}'
    )


def print_statuses(recording):
    total = len(recording.rangings)
    for status in recording.statuses():
        of_status = [ranging for ranging in recording.rangings if ranging.status == status]
        distances = [ranging.distance_mm for ranging in of_status]
        signals = [ranging.signal_per_spad_mcps for ranging in of_status]
        print(
            f'status {status}: {len(distances)} ({100 * len(distances) / total:.1f} %), '
            f'distance min {min(distances)}, median {median(distances):.0f}, '
            f'max {max(distances)} mm; '
            f'signal per SPAD min {min(signals):.4f}, median {median(signals):.4f}, '
            f'max {max(signals):.4f} MCPS; '
            f'longest gap if only it were unclassifiable {milliseconds(recording.longest_gap({status}))}'
        )


def report(recording, unclassifiable):
    rangings = recording.rangings
    if not rangings:
        print('no rangings')
        return
    duration = rangings[-1].time - rangings[0].time
    print(f'rangings: {len(rangings)} in {duration:.1f} s; log lines: {len(recording.log_lines)}')
    for line in recording.log_lines:
        print(f'  {line}')
    print_intervals(recording)
    print_statuses(recording)
    if unclassifiable:
        print(
            f'longest gap between classifiable rangings (unclassifiable {sorted(unclassifiable)}): '
            f'{milliseconds(recording.longest_gap(unclassifiable))}'
        )


def record(port, path, duration_s):
    import serial  # only the recording needs the serial port

    with serial.Serial(port, timeout=READ_TIMEOUT_S) as console, open(path, 'w') as file:
        console.reset_input_buffer()
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            file.write(console.readline().decode(errors='replace'))


def parse_statuses(text):
    return {int(status) for status in text.split(',')}


def main():
    command, *arguments = sys.argv[1:]
    if command == 'record':
        port, path, duration_s = arguments
        record(port, path, float(duration_s))
        unclassifiable = set()
    elif command == 'analyze' and len(arguments) in (1, 2):
        path = arguments[0]
        unclassifiable = parse_statuses(arguments[1]) if len(arguments) == 2 else set()
    else:
        sys.exit(__doc__)
    report(Recording.from_file(path), unclassifiable)


if __name__ == '__main__':
    main()
