"""
Analysis of the encoder AO diagnostic (robot-docs nota 0010): for each duty step, the slots seen
on the analog output (AO) against the slots counted by the firmware, and the charts of the step
responses, one panel per wheel and direction.

Usage: python3 analyze_encoder_ao.py <file.csv> <output prefix>
"""

import sys
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from scipy.optimize import curve_fit

WHEEL_TRAVEL_PER_SLOT = np.pi * 0.065 / 20  # s (m): D = 65 mm, N = 20 (V-02)
AO_THRESHOLD = 2000  # ADC counts, mid-swing of the AO (~0.1 to ~3.1 V)
VOLTS_PER_COUNT = 3.1 / 4095  # 12 dB attenuation, approximate
STEP_S = 3.0
STEADY_FROM_S = 1.0  # the step is steady from here on (tau ~40 ms)
ZOOM_S = (1.0, 1.4)
WHEELS = {'L': 'roda esquerda', 'R': 'roda direita'}
DIRECTIONS = {1: 'frente', -1: 'ré'}
AO_COLUMN = {'L': 1, 'R': 2}
TRACE, SLOT, FIT = '#2a78d6', '#1baf7a', '#52514e'
MUTED, GRID, SURFACE = '#52514e', '#e4e3df', '#fcfcfb'
SHADES = LinearSegmentedColormap.from_list('', ['#bcd5f3', '#0d3a73'])

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'axes.edgecolor': GRID,
    'axes.labelcolor': MUTED, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8, 'font.size': 8,
    'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False,
})


@dataclass
class Run:
    """AO samples, firmware slots (time, count) per wheel and the duty steps, times in seconds."""

    ao: np.ndarray
    slots: dict
    steps: list  # (start time, duty)

    @classmethod
    def from_file(cls, path):
        ao, slots, duty = [], {'L': [], 'R': []}, []
        with open(path, errors='replace') as file:
            lines = file.readlines()
        for line in lines:
            fields = line.strip().split(',')
            try:
                if fields[0] == 'ao':
                    ao.append((int(fields[1]) / 1e6, int(fields[2]), int(fields[3])))
                elif fields[0] == 'edge':
                    slots[fields[2]].append((int(fields[1]) / 1e6, int(fields[3])))
                elif fields[0] == 'duty':
                    duty.append((int(fields[1]) / 1e6, float(fields[2])))
            except (IndexError, ValueError):
                continue  # a line cut by the console
        steps = [(time, value) for time, value in duty if value != 0.0]
        return cls(np.array(ao), {w: np.array(s) for w, s in slots.items()}, steps)

    def ao_between(self, start, end):
        return self.ao[(self.ao[:, 0] >= start) & (self.ao[:, 0] < end)]

    def slots_between(self, wheel, start, end):
        slots = self.slots[wheel]
        return slots[(slots[:, 0] >= start) & (slots[:, 0] < end)]


def ao_slot_times(samples, wheel):
    """Instants where the AO crosses AO_THRESHOLD upwards: one per slot."""
    high = samples[:, AO_COLUMN[wheel]] > AO_THRESHOLD
    return samples[1:, 0][~high[:-1] & high[1:]]


def speed(times):
    return WHEEL_TRAVEL_PER_SLOT / np.median(np.diff(times)) if len(times) > 2 else 0.0


def first_order(t, steady, tau, dead):
    return np.where(t > dead, steady * (1 - np.exp(-(t - dead) / tau)), 0.0)


def compare_counts(run):
    print('step     wheel  AO slots  firmware  v_AO   v_firmware (m/s)  max count jump')
    for start, duty in run.steps:
        steady = (start + STEADY_FROM_S, start + STEP_S)
        samples = run.ao_between(*steady)
        for wheel in WHEELS:
            ao_slots = ao_slot_times(samples, wheel)
            fw = run.slots_between(wheel, *steady)
            if len(ao_slots) < 3 or len(fw) < 3:
                print(f'{duty:+.3f}  {wheel}      wheel did not turn')
                continue
            fw_count = int(fw[-1, 1] - fw[0, 1])
            print(
                f'{duty:+.3f}  {wheel}      {len(ao_slots) - 1:8d}  {fw_count:8d}  '
                f'{speed(ao_slots):.3f}  {speed(fw[:, 0]):.3f}             '
                f'{int(np.diff(fw[:, 1]).max())}'
            )


def panels(title):
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), sharex=True, sharey=True)
    fig.suptitle(title, x=0.01, ha='left')
    layout = [(w, d) for w in WHEELS for d in DIRECTIONS]
    for ax, (wheel, direction) in zip(axes.flat, layout):
        ax.set_title(f'{WHEELS[wheel]}, {DIRECTIONS[direction]}', loc='left', fontsize=9)
    return fig, axes, layout


def steps_of(run, direction):
    return [(start, duty) for start, duty in run.steps if np.sign(duty) == direction]


def plot_ao(run, prefix):
    for wheel, wheel_name in WHEELS.items():
        for direction, direction_name in DIRECTIONS.items():
            chosen = steps_of(run, direction)
            fig, axes = plt.subplots(len(chosen), 1, figsize=(13, 1.9 * len(chosen)),
                                     sharey=True)
            for ax, (start, duty) in zip(axes, chosen):
                window = (start + ZOOM_S[0], start + ZOOM_S[1])
                samples = run.ao_between(*window)
                ax.plot((samples[:, 0] - window[0]) * 1e3,
                        samples[:, AO_COLUMN[wheel]] * VOLTS_PER_COUNT, color=TRACE, lw=1)
                for time in run.slots_between(wheel, *window)[:, 0]:
                    ax.axvline((time - window[0]) * 1e3, color=SLOT, lw=1.2)
                ax.set_title(f'duty {duty:+.3f}', loc='left', fontsize=8)
                ax.set_xlim(0, (window[1] - window[0]) * 1e3)
                ax.set_ylabel('AO (V)')
            axes[-1].set_xlabel('tempo (ms), de 1,0 a 1,4 s depois do degrau')
            legend = [plt.Line2D([], [], color=TRACE, label='AO (satura em ~3,1 V)'),
                      plt.Line2D([], [], color=SLOT, label='fenda contada pelo firmware')]
            fig.legend(handles=legend, loc='upper right', ncol=2)
            fig.suptitle(f'Saída analógica do encoder e fendas contadas: {wheel_name}, '
                         f'{direction_name}', x=0.01, ha='left')
            fig.tight_layout(rect=(0, 0, 1, 0.97))
            name = direction_name.replace('é', 'e')
            fig.savefig(f'{prefix}_ao_{wheel}_{name}.png', dpi=110)
            plt.close(fig)


def plot_steady(run, prefix):
    fig, axes, layout = panels('Regime nos degraus: firmware (pontos) e AO (x); reta até 0,48')
    for ax, (wheel, direction) in zip(axes.flat, layout):
        duties, fw_speeds, ao_speeds = [], [], []
        for start, duty in steps_of(run, direction):
            steady = (start + STEADY_FROM_S, start + STEP_S)
            duties.append(abs(duty))
            fw_speeds.append(speed(run.slots_between(wheel, *steady)[:, 0]))
            ao_speeds.append(speed(ao_slot_times(run.ao_between(*steady), wheel)))
        duties, fw_speeds = np.array(duties), np.array(fw_speeds)
        fit = (duties <= 0.48) & (fw_speeds > 0)
        gain, offset = np.polyfit(duties[fit], fw_speeds[fit], 1)
        line = np.linspace(-offset / gain, max(duties) + 0.02, 2)
        ax.plot(line, gain * line + offset, color=FIT, lw=1,
                label=f'v = {gain:.2f} · (|duty| − {-offset / gain:.3f})')
        ax.plot(duties, fw_speeds, 'o', ms=7, color=TRACE, label='firmware')
        ax.plot(duties, ao_speeds, 'x', ms=8, color=SLOT, label='AO')
        ax.legend(loc='upper left')
    for ax in axes[1]:
        ax.set_xlabel('|duty|')
    for ax in axes[:, 0]:
        ax.set_ylabel('velocidade em regime (m/s)')
    fig.tight_layout()
    fig.savefig(f'{prefix}_regime.png', dpi=130)
    plt.close(fig)


def plot_steps(run, prefix):
    fig, axes, layout = panels(
        'Resposta ao degrau (pontos: velocidade entre fendas; linha: 1ª ordem com atraso)'
    )
    for ax, (wheel, direction) in zip(axes.flat, layout):
        chosen = steps_of(run, direction)
        taus = []
        for k, (start, duty) in enumerate(chosen):
            times = run.slots_between(wheel, start, start + STEP_S)[:, 0]
            if len(times) < 6:
                continue
            t = (times[1:] + times[:-1]) / 2 - start
            v = WHEEL_TRAVEL_PER_SLOT / np.diff(times)
            color = SHADES(k / (len(chosen) - 1))
            ax.plot(t, v, 'o', ms=3, color=color, label=f'duty {abs(duty):.3f}')
            if abs(duty) >= 0.3:
                p, _ = curve_fit(first_order, t, v, p0=[np.median(v[-10:]), 0.04, 0.02],
                                 bounds=([0, 0.005, 0], [3, 1, 0.3]))
                curve = np.linspace(0, 0.5, 200)
                ax.plot(curve, first_order(curve, *p), color=color, lw=1.2)
                taus.append(f'{p[1] * 1e3:.0f}')
        ax.set_title(f'{WHEELS[wheel]}, {DIRECTIONS[direction]}  ·  τ (|duty| ≥ 0,3) = '
                     f'{" / ".join(taus)} ms', loc='left', fontsize=9)
        ax.set_xlim(0, 0.5)
    axes[0][0].legend(loc='lower right', fontsize=7)
    for ax in axes[1]:
        ax.set_xlabel('tempo desde o degrau (s)')
    for ax in axes[:, 0]:
        ax.set_ylabel('velocidade (m/s)')
    fig.tight_layout()
    fig.savefig(f'{prefix}_degraus.png', dpi=130)
    plt.close(fig)


def plot_brakes(run, prefix):
    fig, axes, layout = panels('Freio total depois de cada degrau (velocidade entre fendas)')
    for ax, (wheel, direction) in zip(axes.flat, layout):
        chosen = steps_of(run, direction)
        for k, (start, duty) in enumerate(chosen):
            brake = start + STEP_S
            times = np.concatenate([run.slots_between(wheel, brake - 0.1, brake)[:, 0][-3:],
                                    run.slots_between(wheel, brake, brake + 2.0)[:, 0]])
            if len(times) < 2:
                continue
            ax.plot((times[1:] + times[:-1]) / 2 - brake, WHEEL_TRAVEL_PER_SLOT / np.diff(times),
                    'o-', ms=3, lw=1, color=SHADES(k / (len(chosen) - 1)),
                    label=f'duty {abs(duty):.3f}')
        ax.set_xlim(-0.06, 0.2)
    axes[0][0].legend(loc='upper right', fontsize=7)
    for ax in axes[1]:
        ax.set_xlabel('tempo desde o início do freio (s)')
    for ax in axes[:, 0]:
        ax.set_ylabel('velocidade (m/s)')
    fig.tight_layout()
    fig.savefig(f'{prefix}_freio.png', dpi=130)
    plt.close(fig)


def main():
    run = Run.from_file(sys.argv[1])
    prefix = sys.argv[2]
    compare_counts(run)
    plot_ao(run, prefix)
    plot_steady(run, prefix)
    plot_steps(run, prefix)
    plot_brakes(run, prefix)


if __name__ == '__main__':
    main()
