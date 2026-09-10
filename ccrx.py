#!/usr/bin/env python3
"""
ccrx.py - Proflame2 fireplace remote RX: captures the physical remote's
315MHz transmissions via a CC1101 + pigpio hardware-timestamped GPIO
capture, and decodes them in software (this protocol has no real preamble,
so CC1101's own hardware sync-word detection was never reliable - see
"Design history" below). Standalone script, separate from fpctrl.py.

Pin map (matches test_power_on.py / current fpctrl.py hardware generation):
  RAD_EN (LDO enable)     GPIO27
  PWRGD  (LDO power-good) GPIO22
  CC1101 GDO-0            GPIO5   (async serial data output)
  CC1101 CHIPSEL          GPIO8   (hardware SPI0 CE0)

Requires pigpiod running:
  sudo apt install pigpio python3-pigpio
  sudo systemctl enable --now pigpiod

Usage:
  python3 ccrx.py [--listen-seconds N] [--verbose]

--- Design history (condensed) ---
Register tuning verified against a real SmartRF Studio export (CC1101,
ASK/OOK, 2.4kBaud, 314.972687MHz) - two real bugs found and fixed this way:
MDMCFG2 SYNC_MODE was set to "30/32 doubled sync" instead of "16/16 single
match" (this protocol's repeats have a gap between them, so the doubled
pattern never occurs on air), and MDMCFG4's channel filter was less than
half SmartRF's recommended bandwidth, distorting pulse timing. A later
attempt to cherry-pick individual AGC/FREND1 registers from Flipper Zero's
firmware (which does control this exact remote, verified via its captured
.sub files) caused receiver instability - AGC target registers are
calibrated as a matched SET relative to a specific filter bandwidth, not
independently swappable across different sources' configs.

Even with all registers verified correct, CC1101's own hardware sync-word
detection never reliably fired - root cause: the protocol's "S" sync
symbol is only 2 chip-periods of constant level (no transition inside it),
and repeats are separated by a plain zero-gap, not a real alternating
preamble. The chip's own bit-clock-recovery has nothing to lock onto
before it's asked to start correlating.

The fix that actually worked: capture the raw demodulated bitstream via
GDO0 in CC1101's asynchronous serial mode, and do sync-word search +
decode entirely in software. First attempt used a plain Python busy-loop,
which reliably resolved timing (validated via direct benchmarking) but
gap-distribution analysis on real captures showed multi-millisecond
scheduler-preemption stalls that silently corrupted the run data - a gap
that large gets merged into an adjacent run's duration with no way to know
real information went missing inside it. Fixed by switching capture to
pigpio, whose daemon (pigpiod) timestamps GPIO edges via hardware DMA in a
separate process, immune to this script's own scheduling.

The remaining piece was a systematic bit-period miscalibration: even a
carefully pooled/averaged bit-period constant wasn't accurate enough for
any single specific packet, because real oscillator/measurement conditions
vary capture to capture by enough to matter over a 182-bit window. Fixed
via two_pass_locally_calibrated_search() below: bootstrap alignment with a
rough global period, confirm the first two words are structurally valid,
then recalibrate the actual bit period from that confirmed-good prefix's
real elapsed time before decoding the rest of the packet. This is what
made real, complete, checksum-valid decodes of live remote presses work
end to end - confirmed via multiple independent repeats within a burst
agreeing with each other, and decoded state changes matching real button
presses (flame/fan/light stepping, power/backburner/pilot toggling) across
several live sessions.
"""

import argparse
import json
import os
import socket as socketlib
import sys
import time

import spidev
import RPi.GPIO as GPIO

# proflame2_protocol.py's location relative to ccrx.py has moved more than
# once - check the script's own directory first, then its parent, and add
# whichever one actually contains proflame2_protocol.py. Based on this
# script's own location (__file__), not the current working directory.
_this_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_this_dir)
if os.path.exists(os.path.join(_this_dir, "proflame2_protocol.py")):
    sys.path.insert(0, _this_dir)
elif os.path.exists(os.path.join(_parent_dir, "proflame2_protocol.py")):
    sys.path.insert(0, _parent_dir)
else:
    print(f"WARNING: proflame2_protocol.py not found in {_this_dir} or "
          f"{_parent_dir} - the import below will likely fail.", file=sys.stderr)

from proflame2_protocol import (
    ChecksumConstants,
    raw_bits_to_symbols,
    decode_word,
    decode_packet_symbols,
    command_bytes_to_state,
)
from device_config import (
    RAD_EN, PWRGD, GDO0, XOSC_HZ, TARGET_FREQ_HZ,
    SERIAL_NUMBER, CHECKSUM_C1, CHECKSUM_D1, CHECKSUM_C2, CHECKSUM_D2,
)

CHECKSUM = ChecksumConstants(c1=CHECKSUM_C1, d1=CHECKSUM_D1, c2=CHECKSUM_C2, d2=CHECKSUM_D2)

NPDAEMON_SOCK = "/run/npdaemon/npdaemon.sock"
IDLE_COLOR = [0, 0, 60]  # matches fpctrl.py's idle state
IDLE_BRIGHTNESS = 40


def np_send(payload: dict):
    try:
        with socketlib.socket(socketlib.AF_UNIX, socketlib.SOCK_STREAM) as s:
            s.settimeout(1.0)
            s.connect(NPDAEMON_SOCK)
            s.sendall(json.dumps(payload).encode() + b"\n")
    except Exception:
        pass


def np_listening():
    np_send({"effect": "pulse", "color": [200, 120, 0], "brightness": 50, "speed": 1.0, "duration": 3.0})


def np_rssi_scanning():
    np_send({"effect": "pulse", "color": [120, 0, 200], "brightness": 50, "speed": 1.0, "duration": 3.0})


def np_success():
    np_send({"effect": "pulse", "color": [0, 255, 0], "brightness": 150, "speed": 3.0, "duration": 0.6, "override": True})
    np_listening()


def np_error():
    np_send({"effect": "pulse", "color": [255, 0, 0], "brightness": 150, "speed": 3.0, "duration": 0.6, "override": True})
    np_listening()


def np_idle():
    np_send({"effect": "solid", "color": IDLE_COLOR, "brightness": IDLE_BRIGHTNESS})

# --- CC1101 strobe/register addresses ---
SRES = 0x30
SCAL = 0x33
SRX = 0x34
SIDLE = 0x36
SFRX = 0x3A

REG_IOCFG0 = 0x02
REG_FIFOTHR = 0x03
REG_PKTCTRL1 = 0x07
REG_PKTCTRL0 = 0x08
REG_FSCTRL1 = 0x0B
REG_FREQ2 = 0x0D
REG_FREQ1 = 0x0E
REG_FREQ0 = 0x0F
REG_MDMCFG4 = 0x10
REG_MDMCFG3 = 0x11
REG_MDMCFG2 = 0x12
REG_MDMCFG1 = 0x13
REG_MDMCFG0 = 0x14
REG_DEVIATN = 0x15
REG_MCSM1 = 0x17
REG_MCSM0 = 0x18
REG_FOCCFG = 0x19
REG_AGCCTRL2 = 0x1B
REG_AGCCTRL1 = 0x1C
REG_AGCCTRL0 = 0x1D
REG_FREND1 = 0x21
REG_FSCAL3 = 0x23
REG_FSCAL2 = 0x24
REG_FSCAL1 = 0x25
REG_FSCAL0 = 0x26
REG_TEST2 = 0x2C
REG_TEST1 = 0x2D
REG_TEST0 = 0x2E

RSSI = 0x34
MARCSTATE = 0x35

# Sync word: computed from this device's verified serial number (0xA3D502).
# First 16 raw Manchester bits of Word 1 (Serial1 byte 0xA3, pad=1): S +
# guard + top 6 data bits. Not used for hardware sync detection any more
# (see design history above) - used as the software search target instead.
SYNC_RAW_BITS = "1110100110010101"  # = 0xE995

PACKET_RAW_BITS = 182  # 7 words x 13 symbols x 2 raw bits/symbol

# Empirically calibrated from real pigpio-captured data (weighted least-
# squares over ~1300 known 1-bit/2-bit runs) - theoretical 1e6/2400=
# 416.667us was measurably off. Only used as a bootstrap starting point for
# two_pass_locally_calibrated_search()'s local recalibration.
BIT_PERIOD_US = 411.206

# Register table - verified against SmartRF Studio (see design history).
# FREQ/MDMCFG3/4 (frequency + baud rate) match cc1101_tx.py exactly.
CC1101_RX_CONFIG = [
    (REG_IOCFG0, 0x06),
    (REG_FIFOTHR, 0x47),
    (REG_PKTCTRL1, 0x00),
    (REG_PKTCTRL0, 0x00),
    (REG_FSCTRL1, 0x06),
    (REG_MDMCFG4, 0xC6),     # CHANBW=101.5625kHz (SmartRF verified)
    (REG_MDMCFG3, 0x83),     # 2399.5 baud (must match TX)
    (REG_MDMCFG2, 0x32),     # DEM_DCFILT_OFF=0, ASK/OOK, SYNC_MODE=010 (unused in async mode, overridden below)
    (REG_MDMCFG1, 0x00),
    (REG_MDMCFG0, 0xF8),
    (REG_DEVIATN, 0x00),     # unused for OOK
    (REG_MCSM1, 0x00),
    (REG_MCSM0, 0x04),       # manual calibration (matches TX's proven pattern)
    (REG_FOCCFG, 0x16),      # SmartRF verified
    (REG_AGCCTRL2, 0x43),    # SmartRF verified
    (REG_AGCCTRL1, 0x49),    # SmartRF verified
    (REG_AGCCTRL0, 0x91),    # CC1101 reset default (not in SmartRF export)
    (REG_FREND1, 0x56),      # CC1101 reset default (not in SmartRF export)
    (REG_FSCAL3, 0xE9),      # SmartRF verified
    (REG_FSCAL2, 0x2A),
    (REG_FSCAL1, 0x00),
    (REG_FSCAL0, 0x1F),      # SmartRF verified
    (REG_TEST2, 0x81),
    (REG_TEST1, 0x35),
    (REG_TEST0, 0x09),
]


def compute_freq_regs(target_hz=TARGET_FREQ_HZ, xosc_hz=XOSC_HZ):
    freq_word = round(target_hz * (1 << 16) / xosc_hz)
    f2 = (freq_word >> 16) & 0xFF
    f1 = (freq_word >> 8) & 0xFF
    f0 = freq_word & 0xFF
    actual_hz = freq_word * xosc_hz / (1 << 16)
    return f2, f1, f0, actual_hz


class CC1101RX:
    def __init__(self, bus=0, device=0, spi_hz=500_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = spi_hz
        self.spi.mode = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(RAD_EN, GPIO.OUT)
        GPIO.setup(PWRGD, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(GDO0, GPIO.IN)

    def strobe(self, addr):
        return self.spi.xfer2([addr])[0]

    def write_reg(self, addr, value):
        self.spi.xfer2([addr & 0x3F, value])

    def read_status_reg(self, addr):
        return self.spi.xfer2([addr | 0xC0, 0x00])[1]

    def _read_config_reg(self, addr):
        """Single-byte read of a config register (0x00-0x2E range) - uses
        the read bit (0x80) without the burst bit, distinct from
        read_status_reg() which is for the 0x30+ status/strobe range."""
        return self.spi.xfer2([addr | 0x80, 0x00])[1]

    def power_on(self, timeout_s=0.5, extra_settle_s=0.5):
        GPIO.output(RAD_EN, GPIO.HIGH)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if GPIO.input(PWRGD):
                time.sleep(extra_settle_s)
                return True
            time.sleep(0.005)
        return False

    def power_off(self):
        GPIO.output(RAD_EN, GPIO.LOW)

    def get_rssi_dbm(self):
        """Read RSSI. RSSI_OFFSET=74 is the CC1101 datasheet's typical
        value at this data rate; treat absolute dBm as approximate, but a
        clear relative jump when the remote is pressed is what matters."""
        raw = self.read_status_reg(RSSI)
        RSSI_OFFSET = 74
        if raw >= 128:
            return (raw - 256) / 2 - RSSI_OFFSET
        return raw / 2 - RSSI_OFFSET

    def get_marcstate(self):
        return self.read_status_reg(MARCSTATE) & 0x1F

    def rssi_scan(self, seconds=15, interval_s=0.05, cutoff_dbm=None):
        """Diagnostic mode: continuously print RSSI, independent of decode
        success. Useful for troubleshooting signal-strength questions on
        their own - e.g. confirming a weak/distant remote press is still
        landing above the noise floor even before worrying about whether
        it decodes, or checking antenna/placement changes.

        cutoff_dbm: if set, only PRINT samples at or above this threshold
        (baseline noise floor is still recorded for the min/max/spread
        summary either way - this only trims console spam, not the data)."""
        self.strobe(SFRX)
        self.strobe(SRX)
        time.sleep(0.01)
        print(f"RSSI scan for {seconds}s - press the remote and watch for a jump "
              f"(baseline noise floor will vary, but a press should stand out):")
        if cutoff_dbm is not None:
            print(f"Cutoff: only showing samples >= {cutoff_dbm} dBm (baseline noise still "
                  f"counted in the summary below, just not printed line-by-line). "
                  f"Change with --rssi-cutoff.")
        else:
            print(f"Cutoff: none - showing every sample, including baseline noise floor. "
                  f"Use --rssi-cutoff=-95 (or similar) to hide routine noise-floor chatter "
                  f"and only show stronger signals.")
        start = time.time()
        deadline = start + seconds
        baseline_samples = []
        last_pulse = 0.0
        np_rssi_scanning()
        try:
            while time.time() < deadline:
                elapsed = time.time() - start
                dbm = self.get_rssi_dbm()
                marc = self.get_marcstate()
                if marc != 0x0D:  # not RX state - re-arm
                    self.strobe(SFRX)
                    self.strobe(SRX)
                baseline_samples.append(dbm)
                if cutoff_dbm is None or dbm >= cutoff_dbm:
                    bar = "#" * max(0, int(dbm + 100))
                    print(f"  [{elapsed:5.2f}s] {dbm:6.1f} dBm  {bar}")
                if time.time() - last_pulse > 2.5:
                    np_rssi_scanning()
                    last_pulse = time.time()
                time.sleep(interval_s)
        except KeyboardInterrupt:
            print("\n(stopped early by Ctrl-C)")
        if baseline_samples:
            print(f"\nMin={min(baseline_samples):.1f}  Max={max(baseline_samples):.1f}  "
                  f"Spread={max(baseline_samples)-min(baseline_samples):.1f} dB")

    def configure_async_sniffer(self):
        """CC1101 asynchronous serial mode: GDO0 outputs the raw
        demodulated bit level in real time, bypassing the packet engine
        entirely (no hardware sync-word matching - see design history)."""
        self.strobe(SRES)
        time.sleep(0.1)

        async_config = [(a, v) for (a, v) in CC1101_RX_CONFIG
                         if a not in (REG_IOCFG0, REG_MDMCFG2, REG_PKTCTRL0)]
        async_config += [
            (REG_IOCFG0, 0x0D),    # GDO0: Serial Data Output (async serial mode)
            (REG_MDMCFG2, 0x30),   # DEM_DCFILT_OFF=0, ASK/OOK, SYNC_MODE=000 (no packet engine)
            (REG_PKTCTRL0, 0x30),  # PKTFORMAT=11 (asynchronous serial mode)
        ]

        for addr, value in async_config:
            self.write_reg(addr, value)

        f2, f1, f0, actual_freq = compute_freq_regs()
        self.write_reg(REG_FREQ2, f2)
        self.write_reg(REG_FREQ1, f1)
        self.write_reg(REG_FREQ0, f0)

        self.strobe(SIDLE)
        self.strobe(SFRX)
        self.strobe(SCAL)
        time.sleep(0.005)

    def dump_key_registers(self):
        """Read back the registers we actually configure and print them -
        a cheap, decisive check that a silent SPI write failure isn't the
        problem if something ever looks wrong again."""
        regs = {
            "IOCFG0": REG_IOCFG0, "FIFOTHR": REG_FIFOTHR,
            "PKTCTRL1": REG_PKTCTRL1, "PKTCTRL0": REG_PKTCTRL0,
            "FSCTRL1": REG_FSCTRL1,
            "FREQ2": REG_FREQ2, "FREQ1": REG_FREQ1, "FREQ0": REG_FREQ0,
            "MDMCFG4": REG_MDMCFG4, "MDMCFG3": REG_MDMCFG3, "MDMCFG2": REG_MDMCFG2,
            "MDMCFG1": REG_MDMCFG1, "MDMCFG0": REG_MDMCFG0,
            "MCSM1": REG_MCSM1, "MCSM0": REG_MCSM0,
        }
        print("--- Register readback ---")
        for name, addr in regs.items():
            val = self._read_config_reg(addr)
            print(f"  {name:10s} (0x{addr:02X}) = 0x{val:02X}  {val:08b}")
        print("--------------------------")

    def capture(self, listen_seconds=20, trigger_dbm=-85, capture_window_s=0.75,
                max_bursts=50, save_runs_prefix=None, cooldown_s=0.15, verbose=False):
        """Watch RSSI as a trigger; on crossing trigger_dbm, capture GDO0's
        raw async serial output via pigpio's hardware-timestamped edge
        callbacks (immune to this process's own scheduling - see design
        history for why that matters), then decode via
        two_pass_locally_calibrated_search()."""
        import pigpio

        pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError(
                "Could not connect to pigpio daemon. Install and start it first:\n"
                "  sudo apt install pigpio python3-pigpio\n"
                "  sudo systemctl enable --now pigpiod")

        edges = []

        def _on_edge(gpio, level, tick):
            if level in (0, 1):  # ignore level==2 (pigpio watchdog timeout marker)
                edges.append((tick, level))

        cb = pi.callback(GDO0, pigpio.EITHER_EDGE, _on_edge)

        self.strobe(SFRX)
        self.strobe(SRX)
        time.sleep(0.01)

        print(f"Listening for the remote (RSSI > {trigger_dbm} dBm triggers a capture, "
              f"{listen_seconds}s total)...")

        start = time.time()
        bursts_captured = 0
        last_status_print = 0.0
        last_pulse = 0.0
        np_listening()
        try:
            while time.time() - start < listen_seconds and bursts_captured < max_bursts:
                dbm = self.get_rssi_dbm()
                now = time.time()
                if verbose and now - last_status_print > 5.0:
                    print(f"  [{now - start:5.1f}s] listening... RSSI={dbm:.1f} dBm")
                    last_status_print = now
                if dbm < trigger_dbm:
                    if now - last_pulse > 2.5:
                        np_listening()
                        last_pulse = now
                    time.sleep(0.005)
                    continue

                trigger_time = time.time()
                edges.clear()
                time.sleep(capture_window_s)
                captured_edges = list(edges)

                if len(captured_edges) < 2:
                    np_error()
                    bursts_captured += 1
                    time.sleep(cooldown_s)
                    continue

                # pigpio ticks are unsigned 32-bit microsecond counters that
                # wrap every ~71.6 minutes - masking handles wraparound
                # safely for our sub-second capture windows.
                runs = []
                for i in range(1, len(captured_edges)):
                    t0, lvl0 = captured_edges[i - 1]
                    t1, _ = captured_edges[i]
                    dur_us = (t1 - t0) & 0xFFFFFFFF
                    runs.append((lvl0, float(dur_us)))

                already_saved = False
                if save_runs_prefix:
                    fname = f"{save_runs_prefix}_burst{bursts_captured}.json"
                    with open(fname, "w") as f:
                        json.dump(runs, f)
                    already_saved = True

                filtered_runs = filter_glitches(runs)
                valid_hits = two_pass_locally_calibrated_search(filtered_runs)

                elapsed = trigger_time - start
                if valid_hits:
                    states = [command_bytes_to_state(d.command1, d.command2)
                              for _, d, _ in valid_hits]
                    distinct = []
                    for s in states:
                        if s not in distinct:
                            distinct.append(s)

                    if len(distinct) == 1:
                        np_success()
                        print(f"[{elapsed:6.2f}s] RSSI={dbm:.1f}dBm  "
                              f"({len(valid_hits)}/{len(valid_hits)} repeats agree)  "
                              f"{format_state(distinct[0])}")
                    else:
                        np_error()
                        print(f"[{elapsed:6.2f}s] RSSI={dbm:.1f}dBm  "
                              f"*** WARNING: {len(distinct)} DISAGREEING decodes in "
                              f"one burst - possible decode error, not routine ***")
                        for s in distinct:
                            n = states.count(s)
                            print(f"    ({n}/{len(states)}) {format_state(s)}")
                else:
                    np_error()
                    if not already_saved:
                        fname = f"miss_burst{bursts_captured}_{int(time.time())}.json"
                        with open(fname, "w") as f:
                            json.dump(runs, f)
                    print(f"[{elapsed:6.2f}s] RSSI={dbm:.1f}dBm  no valid decode "
                          f"(saved: {fname})")

                last_pulse = time.time()
                bursts_captured += 1
                # Cooldown only needs to be long enough to avoid re-triggering
                # on the tail of the SAME burst, not eat dead time between
                # separate close-together presses (measured real bursts run
                # 570-600ms, close to capture_window_s - see design history).
                time.sleep(cooldown_s)
        finally:
            cb.cancel()
            pi.stop()

    def close(self):
        """Deliberately does NOT call a blanket GPIO.cleanup() - that would
        release RAD_EN back to a floating input, undoing power_off()'s LOW
        and depending on an unverified assumption (does this board's RAD_EN
        net have a pull-down that keeps it LOW while floating?). Instead,
        only PWRGD/GDO0 (pure inputs, safe to release) get cleaned up;
        RAD_EN stays actively driven LOW. That's a hardware register state,
        not tied to this process's lifetime - it persists after exit
        regardless of any pull-resistor assumption, which is strictly
        safer than hoping one exists."""
        self.spi.close()
        GPIO.cleanup([PWRGD, GDO0])


def format_state(state) -> str:
    """Human-readable one-line summary of a decoded FireplaceState."""
    return (
        f"power={'ON' if state.power else 'off'} "
        f"flame={state.flame} "
        f"fan={state.fan} "
        f"light={state.light} "
        f"backburner={'ON' if state.backburner else 'off'} "
        f"pilot={'CPI' if state.pilot_cpi else 'IPI'} "
        f"thermostat={'on' if state.thermostat else 'off'}"
    )


def filter_glitches(runs, min_duration_us=None):
    """Merge out implausibly short runs before quantization. A real
    Manchester chip-bit can't be shorter than one bit period (~411us) -
    anything meaningfully under that is a comparator glitch/noise blip,
    not a real transition. Merging removes the short run and combines its
    two neighbors (necessarily the same level, since runs always
    alternate) into one continuous run.

    Default threshold is 1/3 of a bit period (~137us) - comfortably below
    any real minimum with margin, while above typical single-sample
    measurement noise.
    """
    if min_duration_us is None:
        min_duration_us = BIT_PERIOD_US / 3

    runs = list(runs)
    changed = True
    while changed:
        changed = False
        for i, (level, dur_us) in enumerate(runs):
            if dur_us < min_duration_us and 0 < i < len(runs) - 1:
                prev_level, prev_dur = runs[i - 1]
                next_level, next_dur = runs[i + 1]
                if prev_level == next_level:
                    merged = (prev_level, prev_dur + dur_us + next_dur)
                    runs = runs[:i - 1] + [merged] + runs[i + 2:]
                    changed = True
                    break
    return runs


def quantize_window(window, period, max_bits=PACKET_RAW_BITS):
    """Drift-corrected quantization of a run window at a given bit period:
    tracks CUMULATIVE elapsed time and derives each run's bit count from
    where that puts us on the absolute timeline, so a small error in one
    run self-corrects on the next rather than compounding."""
    bits = []
    cum_time_us = 0.0
    cum_bits = 0
    for level, dur_us in window:
        cum_time_us += dur_us
        target_bits = round(cum_time_us / period)
        n = max(1, target_bits - cum_bits)
        cum_bits += n
        bits.append(str(level) * n)
        if cum_bits >= max_bits:
            break
    return "".join(bits)


def two_pass_locally_calibrated_search(runs, sync_bits: str = SYNC_RAW_BITS,
                                        checksum: ChecksumConstants = None,
                                        global_period: float = None,
                                        max_sync_errors: int = 3):
    """The search that actually decodes real hardware data reliably - see
    module design history for how this was found.

    Two passes per candidate phase (tried at every run boundary, since
    there's no real preamble to establish alignment for us):
      1. Quantize just the first 2 words (52 bits) using global_period,
         confirm they're structurally valid (sync/guard/parity - cheap,
         filters out non-candidates fast).
      2. Calibrate a LOCAL period from exactly how much real elapsed time
         those confirmed-good 52 bits spanned, then re-quantize the ENTIRE
         182-bit packet using that refined, per-occurrence period instead
         of the global constant. A single global period, even carefully
         averaged from real data, isn't precise enough for any one
         specific packet - real timing varies capture to capture by
         enough to matter over a 182-bit window.

    Returns a list of (start_idx, DecodedBurst, local_period_us) for every
    candidate that produced a genuinely valid (checksum-passing) decode.
    """
    if checksum is None:
        checksum = CHECKSUM
    if global_period is None:
        global_period = BIT_PERIOD_US

    results = []
    n_sync = len(sync_bits)

    for start_idx in range(len(runs)):
        window = runs[start_idx:]

        quick = quantize_window(window, global_period, max_bits=n_sync)
        if len(quick) < n_sync:
            continue
        hamming = sum(1 for a, b in zip(quick[:n_sync], sync_bits) if a != b)
        if hamming > max_sync_errors:
            continue

        prefix_raw = quantize_window(window, global_period, max_bits=52)
        if len(prefix_raw) < 52:
            continue
        try:
            prefix_symbols = raw_bits_to_symbols(prefix_raw[:52])
            w0 = decode_word(prefix_symbols[:13])
            w1 = decode_word(prefix_symbols[13:26])
        except ValueError:
            continue
        if not (w0.sync_ok and w0.guard_ok and w0.parity_ok
                and w1.guard_ok and w1.parity_ok):
            continue

        cum_bits = 0
        cum_time_us = 0.0
        for level, dur_us in window:
            cum_time_us += dur_us
            target_bits = round(cum_time_us / global_period)
            n = max(1, target_bits - cum_bits)
            cum_bits += n
            if cum_bits >= 52:
                break
        if cum_bits == 0:
            continue
        local_period = cum_time_us / cum_bits

        full = quantize_window(window, local_period, max_bits=PACKET_RAW_BITS)
        full = full[:PACKET_RAW_BITS]
        if len(full) != PACKET_RAW_BITS:
            continue
        try:
            symbols = raw_bits_to_symbols(full)
            decoded = decode_packet_symbols(symbols, checksum)
            if decoded.valid:
                results.append((start_idx, decoded, local_period))
        except ValueError:
            continue

    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listen-seconds", type=int, default=30,
                         help="How long to listen before exiting (default 30)")
    parser.add_argument("--trigger-dbm", type=float, default=-85.0,
                         help="RSSI threshold to trigger a capture (default -85). Noise floor "
                              "sits around -94 to -99dBm; real presses measured -30 to -85dBm "
                              "depending on distance.")
    parser.add_argument("--capture-window", type=float, default=0.75,
                         help="Seconds to capture per trigger (default 0.75, calibrated from "
                              "measured real burst durations of 570-600ms)")
    parser.add_argument("--cooldown", type=float, default=0.15,
                         help="Seconds to wait after each capture before re-arming (default 0.15)")
    parser.add_argument("--max-bursts", type=int, default=50,
                         help="Max triggered bursts before stopping early (default 50)")
    parser.add_argument("--save-runs", type=str, default=None,
                         help="Save the complete run list for EVERY burst (hit or miss) to "
                              "'{prefix}_burstN.json'. Misses are always auto-saved regardless.")
    parser.add_argument("--verbose", action="store_true",
                         help="Print register readback on startup and periodic RSSI status "
                              "while listening (quiet by default).")
    parser.add_argument("--rssi-scan", action="store_true",
                         help="Diagnostic mode: continuously print RSSI, independent of decode "
                              "success. Useful for troubleshooting signal-strength on its own - "
                              "e.g. confirming a weak/distant press still lands above the noise "
                              "floor, or checking antenna/placement changes.")
    parser.add_argument("--rssi-cutoff", type=float, default=None,
                         help="With --rssi-scan: only print samples at or above this dBm "
                              "threshold (e.g. -95 to hide routine noise-floor baseline chatter "
                              "around -99 to -101dBm). Baseline is still counted in the final "
                              "min/max/spread summary either way - this only trims console spam.")
    args = parser.parse_args()

    radio = CC1101RX()
    try:
        print("Powering on radio...")
        if not radio.power_on():
            print("PWRGD never went high - LDO/power sequencing failure. Aborting.")
            sys.exit(1)

        radio.configure_async_sniffer()
        if args.verbose:
            radio.dump_key_registers()

        try:
            if args.rssi_scan:
                radio.rssi_scan(seconds=args.listen_seconds, cutoff_dbm=args.rssi_cutoff)
                return

            radio.capture(listen_seconds=args.listen_seconds, max_bursts=args.max_bursts,
                          trigger_dbm=args.trigger_dbm, save_runs_prefix=args.save_runs,
                          capture_window_s=args.capture_window, cooldown_s=args.cooldown,
                          verbose=args.verbose)
        except KeyboardInterrupt:
            print("\nInterrupted by user - shutting down cleanly...")
    finally:
        np_idle()
        radio.power_off()
        radio.close()


if __name__ == "__main__":
    main()