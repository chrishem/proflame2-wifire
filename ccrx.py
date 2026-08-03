#!/usr/bin/env python3
"""
ccrx.py - STANDALONE bench test for hardware sync-word RX (Option B from
the RX subsystem design discussion). Deliberately a separate script from
fpctrl.py, not a daemon - this is for proving the approach works on the
bench before any integration decision gets made. It leverages
proflame2_protocol.py (decode_packet_symbols et al.) but does not import
or touch fpctrl.py, cc1101_tx.py's TX path, or fireplace_state.json.

Pin map (matches test_power_on.py / current fpctrl.py hardware generation):
  RAD_EN (LDO enable)   GPIO27
  PWRGD  (LDO power-good) GPIO22
  CC1101 GDO-0           GPIO5   (configured below as sync-word interrupt)
  CC1101 CHIPSEL         GPIO8   (hardware SPI0 CE0)

*** FIXED BUGS (was: 0/0 captures despite confirmed RF reception) ***
Two real bugs found via bench diagnostics, in order discovered:
1. MDMCFG2 SYNC_MODE was 011=3 ("30/32 sync word bits detected" - requires
   the 16-bit sync pattern twice back-to-back), not 010=2 ("16/16" - a
   single match). This protocol's repeats have a 12-bit zero gap between
   them, so the doubled pattern never occurs on air. Fixed to SYNC_MODE=2.
2. MDMCFG2.DEM_DCFILT_OFF (bit 7) was left at its default 0 (DC-blocking
   filter enabled). A --raw-capture bypassing the packet engine showed the
   real symptom: every 'carrier on' period was truncated to a short ~250us
   blip regardless of true duration, then decayed to LOW - the textbook
   signature of a high-pass (DC-blocking) filter unable to pass a sustained
   level. Our OOK stream has long constant-level runs (the 'S' sync symbol
   alone is 833us sustained-high) that this filter destroys. Fixed to
   DEM_DCFILT_OFF=1 (disabled) - appropriate for OOK, per the CC1101
   datasheet's own register description (0=enable/better sensitivity,
   1=disable), unlike FSK/GFSK where the default is usually left enabled.
Both confirmed via RSSI scan (proved RF reception was fine throughout -
neither bug was an antenna/wiring/frequency problem) and --raw-capture
(showed the actual demodulated waveform shape, which pointed straight at
DCFILT rather than continued guessing at sync-word values).

*** SmartRF Studio cross-check (RX-tuning registers) ***
Ported analog/demod register values from a real SmartRF Studio export
(CC1101, ASK/OOK, 2.4 kBaud, 314.972687 MHz) rather than continuing to guess:
  - MDMCFG4: 0xF6 -> 0xC6. Real bug - our RX channel filter bandwidth was
    58.0kHz, less than half SmartRF's recommended 101.5625kHz for this data
    rate. A too-narrow filter distorts pulse timing - a strong candidate for
    the short/inconsistent pulse widths seen in earlier --raw-capture runs.
  - MDMCFG2 DEM_DCFILT_OFF: reverted my own earlier guess (set to 1/disabled)
    back to 0/enabled - SmartRF's computed value for this exact modulation
    contradicts that guess. Own sync word/SYNC_MODE kept as-is (SmartRF's
    export bakes in its own generic sync word/framing, not applicable here).
  - AGCCTRL1: 0x40 -> 0x49. FSCAL3: 0xEA -> 0xE9. FSCAL0: 0x11 -> 0x1F.
  - FOCCFG, AGCCTRL2, FSCTRL1, MDMCFG3 matched SmartRF exactly already - no
    change, good confirmation those particular earlier guesses were right.
  - AGCCTRL0/FREND1/TEST0 don't appear in the SmartRF export at all, meaning
    they're left at CC1101's reset default - which is what we already had.

*** Flipper Zero firmware cross-check - TRIED AND REVERTED ***
Pulled lib/subghz/devices/cc1101_configs.c from flipperdevices/flipperzero-
firmware (dev branch) - the actual register presets Flipper's "Read RAW"
uses, proven on real hardware against this exact remote (the AM270 preset
specifically - confirmed as what captured the original .sub files this
project's checksum constants were derived from). Tried swapping in
AGCCTRL0/1/2, FOCCFG, and FREND1 from that source over our SmartRF-derived
values for those five registers - REVERTED after --raw-capture showed
clearly worse results (erratic pulses, including physically-impossible
tens-of-milliseconds "stuck high" runs - receiver instability/saturation,
not real signal).
Root cause understood: AGC target/threshold registers are calibrated
relative to a SPECIFIC channel filter bandwidth - they're not independent,
freestanding values. Flipper's AGC set was tuned together with their
270kHz filter (MDMCFG4). We kept our own narrower 101.5kHz filter (from
SmartRF) while swapping in AGC values tuned for a different filter width -
an internally-inconsistent mix, not a coherent verified config. Lesson:
don't cherry-pick individual registers across two different sources'
matched sets - a "verified" config is verified as a WHOLE, not register by
register. Reverted FOCCFG/AGCCTRL0/1/2/FREND1 back to the fully
self-consistent SmartRF Studio values, which were tuned together for our
exact 2400 baud / 101.5kHz filter combination and gave genuinely clean
raw-capture data (~417us/~833us pulse clusters, matching real Manchester
chip-bit timing).
FREND1 specifically: still worth someday verifying independently (it truly
was never confirmed - SmartRF's export omitted it, implying reset default,
which is what we're back to at 0x56) - but not by borrowing Flipper's
value wholesale again without also matching their filter bandwidth.

Usage:
  python3 ccrx.py [--listen-seconds N]

No root required (unlike test_power_on.py) - this only touches spidev/GPIO,
not rpi_ws281x/DMA.
"""

import argparse
import json
import os
import sys
import time

import spidev
import RPi.GPIO as GPIO

# proflame2_protocol.py's location relative to ccrx.py has moved more than
# once (same directory, then a subdirectory, now back to the same
# directory) - rather than hardcode one assumption, check the script's own
# directory first, then its parent, and add whichever one actually
# contains proflame2_protocol.py. Based on this script's own location
# (__file__), not the current working directory, so it works no matter
# where ccrx.py is run from.
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
    bytes_to_bits,
    raw_bits_to_symbols,
    decode_word,
    decode_packet_symbols,
    command_bytes_to_state,
)

RAD_EN = 27
PWRGD = 22
GDO0 = 5

SERIAL_NUMBER = 0xA3D502
CHECKSUM = ChecksumConstants(c1=0x7, d1=0x5, c2=0x4, d2=0xD)

# --- CC1101 strobe/register addresses (shared with cc1101_tx.py's table) ---
SRES = 0x30
SCAL = 0x33
SRX = 0x34
SIDLE = 0x36
SFRX = 0x3A

REG_IOCFG0 = 0x02
REG_FIFOTHR = 0x03
REG_PKTLEN = 0x06
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
REG_SYNC1 = 0x04
REG_SYNC0 = 0x05

MARCSTATE = 0x35
RXBYTES = 0x3B
RXFIFO = 0x3F
RSSI = 0x34

XOSC_HZ = 26_000_000
TARGET_FREQ_HZ = 314_973_000
FIFO_SIZE = 64

# --- Sync word: computed from this device's verified serial number
# (0xA3D502) - see the design discussion. First 16 raw Manchester bits of
# Word 1 (Serial1 byte 0xA3, pad=1): S + guard + top 6 data bits.
SYNC_RAW_BITS = "1110100110010101"  # = 0xE995
SYNC1_VAL = 0xE9
SYNC0_VAL = 0x95

# One packet is 182 raw bits (7 words x 26). The hardware sync consumes the
# first 16 of those (matched against SYNC1/SYNC0, NOT delivered to RXFIFO).
# Remaining bits to capture: 182 - 16 = 166 -> ceil to 21 bytes (168 bits;
# the last 2 captured bits are spillover into the next symbol and are
# trimmed before decode).
REMAINING_BITS = 182 - len(SYNC_RAW_BITS)
PKTLEN_BYTES = -(-REMAINING_BITS // 8)  # ceil division = 21

# Register table. FREQ/MDMCFG3/4 (frequency + baud rate) match cc1101_tx.py
# exactly - same radio, same air rate, must agree with the TX side.
# UNVERIFIED (see module docstring): FOCCFG, AGCCTRL0/1/2 - starting points
# only, not ported from a tested reference.
CC1101_RX_CONFIG = [
    (REG_IOCFG0, 0x06),      # GDO0: asserts on sync word match, deasserts at packet end
    (REG_FIFOTHR, 0x47),
    (REG_PKTCTRL1, 0x00),
    (REG_PKTCTRL0, 0x00),    # fixed length, CRC off, whitening off (matches TX - raw payload)
    (REG_FSCTRL1, 0x06),
    (REG_MDMCFG4, 0xC6),     # DRATE_E=6 -> 2400 baud, CHANBW=101.5625kHz (SmartRF Studio verified - was 0xF6/58.0kHz, too narrow)
    (REG_MDMCFG3, 0x83),     # DRATE_M -> 2399.5 baud (must match TX)
    (REG_MDMCFG2, 0x32),      # DEM_DCFILT_OFF=0 (enabled - SmartRF Studio verified; my earlier guess to disable this was wrong), ASK/OOK, Manchester off, SYNC_MODE=010 (16/16)
    (REG_MDMCFG1, 0x00),
    (REG_MDMCFG0, 0xF8),
    (REG_DEVIATN, 0x00),     # unused for OOK
    (REG_MCSM1, 0x00),       # RXOFF_MODE=IDLE after packet, no CCA
    (REG_MCSM0, 0x04),       # manual calibration (matches TX's proven pattern)
    (REG_FOCCFG, 0x16),      # REVERTED to SmartRF (Flipper's 0x18 caused instability - see docstring)
    (REG_AGCCTRL2, 0x43),    # REVERTED to SmartRF
    (REG_AGCCTRL1, 0x49),    # REVERTED to SmartRF
    (REG_AGCCTRL0, 0x91),    # REVERTED to SmartRF/assumed-default
    (REG_FREND1, 0x56),      # REVERTED to assumed default (Flipper's 0xB6 was part of the same broken mix)
    (REG_FSCAL3, 0xE9),      # SmartRF Studio verified (was 0xEA)
    (REG_FSCAL2, 0x2A),
    (REG_FSCAL1, 0x00),
    (REG_FSCAL0, 0x1F),      # SmartRF Studio verified (was 0x11)
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

    def read_burst(self, addr, n):
        return bytes(self.spi.xfer2([addr | 0xC0] + [0] * n)[1:])

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

    def configure_ook_rx(self):
        self.strobe(SRES)
        time.sleep(0.1)

        for addr, value in CC1101_RX_CONFIG:
            self.write_reg(addr, value)

        f2, f1, f0, actual_freq = compute_freq_regs()
        self.write_reg(REG_FREQ2, f2)
        self.write_reg(REG_FREQ1, f1)
        self.write_reg(REG_FREQ0, f0)

        self.write_reg(REG_SYNC1, SYNC1_VAL)
        self.write_reg(REG_SYNC0, SYNC0_VAL)
        self.write_reg(REG_PKTLEN, PKTLEN_BYTES)

        self.strobe(SIDLE)
        self.strobe(SFRX)
        self.strobe(SCAL)
        time.sleep(0.005)

        return {"target_freq_hz": TARGET_FREQ_HZ, "actual_freq_hz": actual_freq,
                "sync": f"0x{SYNC1_VAL:02X}{SYNC0_VAL:02X}", "pktlen_bytes": PKTLEN_BYTES}

    def get_marcstate(self):
        return self.read_status_reg(MARCSTATE) & 0x1F

    def get_rxbytes(self):
        val = self.read_status_reg(RXBYTES)
        return val & 0x7F, bool(val & 0x80)  # (count, overflow)

    def get_rssi_dbm(self):
        """Read RSSI regardless of sync-word match - bypasses packet engine
        entirely, so it isolates 'is the radio receiving anything at all'
        from 'does the sync word match'. RSSI_OFFSET=74 is the CC1101
        datasheet's typical value at this data rate; treat absolute dBm as
        approximate, but a clear RELATIVE jump when the remote is pressed
        is the useful signal here, not the exact number."""
        raw = self.read_status_reg(RSSI)
        RSSI_OFFSET = 74
        if raw >= 128:
            return (raw - 256) / 2 - RSSI_OFFSET
        return raw / 2 - RSSI_OFFSET

    def rssi_scan(self, seconds=15, interval_s=0.05):
        """Diagnostic mode: continuously print RSSI, ignoring sync-word
        matching entirely. Press the remote and watch for a jump - confirms
        (or rules out) the RF front-end/antenna/frequency independent of
        whether sync-word detection is working."""
        self.strobe(SFRX)
        self.strobe(SRX)
        time.sleep(0.01)
        print(f"RSSI scan for {seconds}s - press the remote and watch for a jump "
              f"(baseline noise floor will vary, but a press should stand out):")
        start = time.time()
        deadline = start + seconds
        baseline_samples = []
        while time.time() < deadline:
            elapsed = time.time() - start
            dbm = self.get_rssi_dbm()
            marc = self.get_marcstate()
            if marc != 0x0D:  # not RX state - re-arm
                self.strobe(SFRX)
                self.strobe(SRX)
            baseline_samples.append(dbm)
            bar = "#" * max(0, int(dbm + 100))
            print(f"  [{elapsed:5.2f}s] {dbm:6.1f} dBm  MARCSTATE=0x{marc:02X}  {bar}")
            time.sleep(interval_s)
        if baseline_samples:
            print(f"\nMin={min(baseline_samples):.1f}  Max={max(baseline_samples):.1f}  "
                  f"Spread={max(baseline_samples)-min(baseline_samples):.1f} dB")
            print("A spread of only a couple dB across the whole scan (even with "
                  "presses) points at antenna/frequency/wiring, not AGC tuning.")

    def gdo0_watch(self, seconds=20, poll_interval_s=0.0002):
        """Diagnostic mode: tight-poll watch on GDO0 for state transitions,
        independent of wait_for_packet()'s packet-complete timing logic.
        Use this once RSSI confirms the RF path works but packet capture
        still gets 0/0 - it isolates 'does sync-word detection ever fire at
        all' from 'does our fixed-length packet completion logic work
        correctly after it fires'.

        Uses polling, not GPIO.add_event_detect() - that call requires sysfs
        edge-detection permissions this process doesn't have running as a
        non-root user (deliberately, to match this script's no-root design).
        Polling works fine here because GDO0 (IOCFG0=0x06) stays asserted
        for the whole packet duration once sync matches, not a brief pulse -
        a ~0.2ms poll interval can't miss a ~350-460ms-wide assertion."""
        self.strobe(SFRX)
        self.strobe(SRX)

        edges = []
        last_state = GPIO.input(GDO0)
        print(f"Watching GDO0 for sync-detect transitions, {seconds}s "
              f"(poll-based, ~{poll_interval_s*1000:.1f}ms interval). "
              f"Press the remote near the antenna...")
        start = time.time()
        while time.time() - start < seconds:
            state = GPIO.input(GDO0)
            if state == 1 and last_state == 0:
                edges.append(time.time() - start)
            last_state = state
            time.sleep(poll_interval_s)

        if not edges:
            print(f"\n0 rising edges on GDO0 in {seconds}s despite confirmed RF "
                  f"reception - sync word ({SYNC_RAW_BITS} = 0x{SYNC1_VAL:02X}{SYNC0_VAL:02X}) "
                  f"is not matching. Double check MDMCFG2's SYNC_MODE field "
                  f"(bits 2:0) - value 2 (0x32) is genuine 16/16, value 3 "
                  f"(0x33) is 30/32 (requires the sync pattern twice back-to-back, "
                  f"which this protocol's gapped repeats never produce).")
        else:
            print(f"\n{len(edges)} rising edge(s) at (relative to start): "
                  f"{[f'{e:.3f}s' for e in edges]}")
            print("GDO0 IS asserting - sync-word detection works. If packet "
                  "capture still returns 0/0, the bug is in wait_for_packet()'s "
                  "post-sync timing/RXBYTES logic, not the sync word itself.")

    def configure_async_sniffer(self):
        """Bypass the packet engine entirely: no sync-word matching, no
        packet framing. GDO0 outputs the raw demodulated bit level in real
        time (CC1101 asynchronous serial mode). Used to get ground truth on
        what the demodulator actually sees, after two SYNC_MODE attempts
        both got 0/0 despite confirmed strong RSSI - this stops guessing at
        packet-engine register values and looks at the real bitstream."""
        self.strobe(SRES)
        time.sleep(0.1)

        # Same RF-facing analog/demod config as configure_ook_rx() (frequency,
        # baud, AGC, front-end) - only the packet-engine-specific registers
        # (IOCFG0, MDMCFG2, PKTCTRL0) change.
        async_config = list(CC1101_RX_CONFIG)
        async_config = [(a, v) for (a, v) in async_config
                         if a not in (REG_IOCFG0, REG_MDMCFG2, REG_PKTCTRL0)]
        async_config += [
            (REG_IOCFG0, 0x0D),    # GDO0: Serial Data Output (async serial mode)
            (REG_MDMCFG2, 0x30),   # DEM_DCFILT_OFF=0 (enabled - SmartRF verified), ASK/OOK, Manchester off, SYNC_MODE=000 (no preamble/sync - bypass packet engine)
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

    def raw_capture_pigpio(self, listen_seconds=20, trigger_dbm=-85, capture_window_s=0.6,
                            max_bursts=50, save_runs_prefix=None):
        """Like raw_capture(), but uses pigpio's hardware-timestamped edge
        callbacks instead of a Python busy-loop.

        WHY: gap-distribution analysis on real captures directly confirmed
        multi-millisecond stalls in the busy loop (up to ~13.5ms observed) -
        genuine Linux kernel scheduler preemption stealing CPU from our
        process, not something fixable by further Python-level tuning. A
        gap that large silently corrupts the resulting run-length data: if
        the level doesn't change across the gap, it just gets merged into
        one artificially-inflated run, and the quantizer has no way to know
        real information is missing from inside it.

        pigpio's daemon (pigpiod) timestamps GPIO edges via hardware DMA
        sampling in a SEPARATE process, independent of our own process's
        scheduling - even if our callback-handling code is delayed, the
        edge's timestamp was already captured accurately at the true
        moment it happened. This is the real fix for the class of problem
        the busy-loop approach couldn't solve no matter how it was tuned.

        Requires: sudo apt install pigpio python3-pigpio; sudo systemctl
        start pigpiod (or run 'sudo pigpiod' once - the daemon must be
        running before this is called)."""
        import pigpio

        pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError(
                "Could not connect to pigpio daemon. Install and start it first:\n"
                "  sudo apt install pigpio python3-pigpio\n"
                "  sudo systemctl start pigpiod   (or: sudo pigpiod)")

        edges = []

        def _on_edge(gpio, level, tick):
            if level in (0, 1):  # ignore level==2 (pigpio watchdog timeout marker)
                edges.append((tick, level))

        cb = pi.callback(GDO0, pigpio.EITHER_EDGE, _on_edge)

        # BUG FIX: raw_capture() (busy-loop version) explicitly strobes the
        # chip into RX state before its wait loop - this was missing here,
        # leaving the chip idle the whole time and producing a stuck/
        # meaningless RSSI reading (confirmed: -138.0dBm constant, which
        # decodes back to a raw RSSI byte of 0x80 - not real noise floor).
        self.strobe(SFRX)
        self.strobe(SRX)
        time.sleep(0.01)

        print(f"pigpio capture: watching RSSI for a trigger above {trigger_dbm} dBm, "
              f"{listen_seconds}s total, up to {max_bursts} bursts. Press the "
              f"remote near the antenna...")

        start = time.time()
        bursts_captured = 0
        last_status_print = 0.0
        try:
            while time.time() - start < listen_seconds and bursts_captured < max_bursts:
                dbm = self.get_rssi_dbm()
                now = time.time()
                if now - last_status_print > 2.0:
                    print(f"  [{now - start:5.1f}s] listening... current RSSI={dbm:.1f} dBm "
                          f"(need >{trigger_dbm} dBm to trigger)")
                    last_status_print = now
                if dbm < trigger_dbm:
                    time.sleep(0.005)
                    continue

                trigger_time = time.time()
                print(f"\n--- Triggered at {trigger_time - start:.3f}s (RSSI={dbm:.1f} dBm) ---")

                edges.clear()
                time.sleep(capture_window_s)  # daemon accumulates edges in the background
                captured_edges = list(edges)

                if len(captured_edges) < 2:
                    print("  (fewer than 2 edges captured - nothing to decode)")
                    bursts_captured += 1
                    time.sleep(0.5)
                    continue

                # pigpio ticks are unsigned 32-bit microsecond counters that
                # wrap every ~71.6 minutes - masking the difference handles
                # wraparound safely for our sub-second capture windows.
                runs = []
                for i in range(1, len(captured_edges)):
                    t0, lvl0 = captured_edges[i - 1]
                    t1, _ = captured_edges[i]
                    dur_us = (t1 - t0) & 0xFFFFFFFF
                    runs.append((lvl0, float(dur_us)))

                if save_runs_prefix:
                    fname = f"{save_runs_prefix}_burst{bursts_captured}.json"
                    with open(fname, "w") as f:
                        json.dump(runs, f)
                    print(f"Saved {len(runs)} complete runs to {fname}")

                print(f"{len(captured_edges)} hardware-timestamped edges, {len(runs)} runs. "
                      f"Expected bit period at 2400 baud: 416.7us")

                filtered_runs = filter_glitches(runs)
                valid_hits = two_pass_locally_calibrated_search(filtered_runs)
                if valid_hits:
                    print(f"*** TWO-PASS LOCALLY-CALIBRATED SEARCH: {len(valid_hits)} "
                          f"VALID packet(s) found ***")
                    for offset, decoded, local_period in valid_hits:
                        state = command_bytes_to_state(decoded.command1, decoded.command2)
                        print(f"    bit offset {offset} (local period {local_period:.2f}us): "
                              f"serial=0x{decoded.serial_number:06X}")
                        print(f"      {format_state(state)}")
                else:
                    print("Two-pass locally-calibrated search: no valid packet found")
                    print("First 60 runs (level, duration_us):")
                    for level, dur_us in runs[:60]:
                        print(f"  {level}  {dur_us:7.1f}us")
                    if len(runs) > 60:
                        print(f"  ... ({len(runs) - 60} more runs)")

                bursts_captured += 1
                time.sleep(0.5)
        finally:
            cb.cancel()
            pi.stop()

    def raw_capture(self, listen_seconds=20, trigger_dbm=-85, capture_window_s=0.6,
                     max_bursts=50, max_samples=100_000, save_runs_prefix=None):
        """Watch RSSI (proven reliable earlier) as a trigger; the instant it
        crosses trigger_dbm, busy-loop-poll GDO0's raw async serial output
        for capture_window_s and print the run-length-encoded result. This
        is ground truth: the literal bit levels the demodulator produced,
        independent of any packet-engine register we might have wrong.

        save_runs_prefix: if set, writes the COMPLETE run list for every
        burst to '{save_runs_prefix}_burst{N}.json' - the printed output
        only ever shows the first 60 of what's often 600+ runs, so this is
        the only way to get the real, complete data out for offline
        analysis instead of guessing at synthetic noise models.

        NOTE: this used to call time.sleep() between samples, which
        loop_bench proved actually achieves only ~230us/iteration on this
        Pi (vs. the ~17.5us/iteration a bare busy-loop achieves) - nowhere
        near enough resolution for a 416.7us-period signal. Fixed to
        busy-loop instead, matching the validated fast pattern."""
        self.strobe(SFRX)
        self.strobe(SRX)
        time.sleep(0.01)

        print(f"Raw capture: watching RSSI for a trigger above {trigger_dbm} dBm, "
              f"{listen_seconds}s total, up to {max_bursts} bursts. Press the "
              f"remote near the antenna...")

        start = time.time()
        bursts_captured = 0
        while time.time() - start < listen_seconds and bursts_captured < max_bursts:
            dbm = self.get_rssi_dbm()
            if dbm < trigger_dbm:
                time.sleep(0.005)  # coarse polling while idle is fine - only the
                                    # capture window itself needs to be fast
                continue

            trigger_time = time.time()
            print(f"\n--- Triggered at {trigger_time - start:.3f}s (RSSI={dbm:.1f} dBm) ---")

            samples = []
            cap_deadline = trigger_time + capture_window_s
            while time.time() < cap_deadline and len(samples) < max_samples:
                samples.append((time.time(), GPIO.input(GDO0)))
            # NOTE: previously polled MARCSTATE every 500 samples here to
            # detect front-end-overload dropout - removed. That check never
            # fired in any real capture (the overload theory was disproven
            # hours ago), but the SPI transaction itself was a real ~100-
            # 300us stall roughly every 12.5ms, injecting periodic gaps
            # directly into the busy loop - a very plausible source of
            # missed short pulses that no downstream decode algorithm could
            # ever recover from, since the data was never sampled at all.
            # (front-end-overload dropout detection removed - see note above)

            # Run-length encode
            runs = []
            cur_level, cur_start = samples[0][1], samples[0][0]
            for t, level in samples[1:]:
                if level != cur_level:
                    runs.append((cur_level, (t - cur_start) * 1e6))  # (level, duration_us)
                    cur_level, cur_start = level, t
            runs.append((cur_level, (samples[-1][0] - cur_start) * 1e6))

            if save_runs_prefix:
                fname = f"{save_runs_prefix}_burst{bursts_captured}.json"
                with open(fname, "w") as f:
                    json.dump(runs, f)
                print(f"Saved {len(runs)} complete runs to {fname}")

            actual_rate_hz = len(samples) / capture_window_s
            avg_gap_us = 1e6 / actual_rate_hz
            sample_gaps_us = [(samples[k][0] - samples[k-1][0]) * 1e6 for k in range(1, len(samples))]
            max_gap_us = max(sample_gaps_us) if sample_gaps_us else 0
            outlier_threshold_us = avg_gap_us * 5
            outlier_gaps = [g for g in sample_gaps_us if g > outlier_threshold_us]
            print(f"{len(samples)} samples ({actual_rate_hz:.0f} Hz achieved, "
                  f"avg gap {avg_gap_us:.1f}us), {len(runs)} runs. "
                  f"Expected bit period at 2400 baud: 416.7us")
            print(f"Sample gap distribution: max={max_gap_us:.1f}us, "
                  f"{len(outlier_gaps)} outlier gap(s) >{outlier_threshold_us:.0f}us "
                  f"(5x average) - each one is a window where a real pulse "
                  f"could have been completely missed, not just mistimed")
            if outlier_gaps:
                worst = sorted(outlier_gaps, reverse=True)[:5]
                print(f"  worst outlier gaps: {[f'{g:.0f}us' for g in worst]}")

            filtered_runs = filter_glitches(runs)
            valid_hits = two_pass_locally_calibrated_search(filtered_runs)
            if valid_hits:
                print(f"*** TWO-PASS LOCALLY-CALIBRATED SEARCH: {len(valid_hits)} "
                      f"VALID packet(s) found ***")
                for offset, decoded, local_period in valid_hits:
                    state = command_bytes_to_state(decoded.command1, decoded.command2)
                    print(f"    bit offset {offset} (local period {local_period:.2f}us): "
                          f"serial=0x{decoded.serial_number:06X}")
                    print(f"      {format_state(state)}")
            else:
                print("Two-pass locally-calibrated search: no valid packet found "
                      "(note: busy-loop capture is known to have missing-pulse "
                      "corruption from OS scheduler stalls - prefer --pigpio-capture)")
                print("First 60 runs (level, duration_us):")
                for level, dur_us in runs[:60]:
                    print(f"  {level}  {dur_us:7.1f}us")
                if len(runs) > 60:
                    print(f"  ... ({len(runs) - 60} more runs)")

            bursts_captured += 1
            time.sleep(0.5)  # avoid re-triggering on the tail of the same burst

        if bursts_captured == 0:
            print(f"\nNo RSSI trigger above {trigger_dbm} dBm in {listen_seconds}s. "
                  f"Lower trigger_dbm or check the remote is being pressed.")

    def dump_key_registers(self):
        """Read back the registers we actually configure and print them
        against what we intended to write. Cheap, decisive check: if a
        register readback doesn't match what configure_*() wrote, the
        problem is a silent SPI write failure, not an RF/protocol theory."""
        regs = {
            "IOCFG0": REG_IOCFG0, "FIFOTHR": REG_FIFOTHR,
            "SYNC1": REG_SYNC1, "SYNC0": REG_SYNC0,
            "PKTLEN": REG_PKTLEN, "PKTCTRL1": REG_PKTCTRL1, "PKTCTRL0": REG_PKTCTRL0,
            "FSCTRL1": REG_FSCTRL1,
            "FREQ2": REG_FREQ2, "FREQ1": REG_FREQ1, "FREQ0": REG_FREQ0,
            "MDMCFG4": REG_MDMCFG4, "MDMCFG3": REG_MDMCFG3, "MDMCFG2": REG_MDMCFG2,
            "MDMCFG1": REG_MDMCFG1, "MDMCFG0": REG_MDMCFG0,
            "MCSM1": REG_MCSM1, "MCSM0": REG_MCSM0,
        }
        print("--- Register readback ---")
        for name, addr in regs.items():
            val = self.read_status_reg(addr) if addr >= 0x30 else self._read_config_reg(addr)
            print(f"  {name:10s} (0x{addr:02X}) = 0x{val:02X}  {val:08b}")
        print("--------------------------")

    def _read_config_reg(self, addr):
        """Single-byte read of a config register (0x00-0x2E range) - uses
        the read bit (0x80) without the burst bit, distinct from
        read_status_reg() which is for the 0x30+ status/strobe range."""
        return self.spi.xfer2([addr | 0x80, 0x00])[1]

    def loop_bench(self, seconds=2.0, sleep_target_s=0.0001):
        """Two benchmarks: (1) bare tight loop, no sleep - measures raw
        GPIO.input() call overhead. (2) the actual pattern raw_capture()
        uses - GPIO.input() + time.sleep(sleep_target_s) per sample. These
        can differ hugely on non-realtime Linux: a requested 100us sleep can
        take far longer in practice due to scheduler wakeup latency, and (2)
        is what actually matters for judging whether raw_capture()'s
        measured pulse widths reflect the real signal or just this loop's
        achieved sample period."""
        print(f"Benchmark 1/2: bare tight loop (no sleep), {seconds}s...")
        count = 0
        start = time.time()
        deadline = start + seconds
        while time.time() < deadline:
            GPIO.input(GDO0)
            count += 1
        elapsed = time.time() - start
        rate_hz = count / elapsed
        print(f"  {count} iterations in {elapsed:.3f}s = {rate_hz:.0f} Hz, "
              f"~{1e6/rate_hz:.1f}us/iteration (raw call overhead only)")

        print(f"\nBenchmark 2/2: GPIO.input() + time.sleep({sleep_target_s*1e6:.0f}us) "
              f"per sample - the ACTUAL pattern raw_capture() uses, {seconds}s...")
        count2 = 0
        start2 = time.time()
        deadline2 = start2 + seconds
        while time.time() < deadline2:
            GPIO.input(GDO0)
            time.sleep(sleep_target_s)
            count2 += 1
        elapsed2 = time.time() - start2
        rate_hz2 = count2 / elapsed2
        actual_period_us = 1e6 / rate_hz2
        print(f"  {count2} iterations in {elapsed2:.3f}s = {rate_hz2:.0f} Hz, "
              f"~{actual_period_us:.1f}us/iteration actually achieved "
              f"(requested {sleep_target_s*1e6:.0f}us)")

        print(f"\nRequested vs actual: {sleep_target_s*1e6:.0f}us requested, "
              f"{actual_period_us:.1f}us actually achieved "
              f"({actual_period_us/(sleep_target_s*1e6):.1f}x slower than requested).")
        print(f"One raw Manchester chip bit at 2400 baud is 416.7us. If the "
              f"achieved period in Benchmark 2 is a large fraction of that "
              f"(roughly a quarter or more), raw_capture()'s measured pulse "
              f"widths were likely bounded by this sleep-loop's real speed, "
              f"not the true signal shape.")



    def arm_rx(self):
        self.strobe(SFRX)
        self.strobe(SRX)

    def wait_for_packet(self, timeout_s=2.0):
        """Poll GDO0 for sync-detect, then wait for the fixed-length packet
        to complete (GDO0 deasserts). Returns the raw captured bytes, or
        None on timeout."""
        deadline = time.time() + timeout_s

        # Wait for sync detect (GDO0 rising edge)
        while not GPIO.input(GDO0):
            if time.time() > deadline:
                return None
            time.sleep(0.0005)

        # Sync detected - wait for packet complete (GDO0 falls) or enough
        # bytes to appear in RXFIFO, whichever we can observe first.
        packet_deadline = time.time() + 0.5  # 21 bytes @ 2400 baud is ~70ms; generous margin
        while time.time() < packet_deadline:
            count, overflow = self.get_rxbytes()
            if overflow:
                return None
            if count >= PKTLEN_BYTES:
                break
            time.sleep(0.001)
        else:
            return None

        data = self.read_burst(RXFIFO, PKTLEN_BYTES)
        self.strobe(SIDLE)
        return data

    def close(self):
        self.spi.close()
        GPIO.cleanup()


def format_state(state) -> str:
    """Human-readable one-line summary of a decoded FireplaceState, e.g.:
    'power=ON flame=6 fan=1 light=0 backburner=off pilot=CPI'
    Used everywhere a decoded packet gets printed, instead of the raw
    dataclass repr."""
    return (
        f"power={'ON' if state.power else 'off'} "
        f"flame={state.flame} "
        f"fan={state.fan} "
        f"light={state.light} "
        f"backburner={'ON' if state.backburner else 'off'} "
        f"pilot={'CPI' if state.pilot_cpi else 'IPI'} "
        f"thermostat={'on' if state.thermostat else 'off'}"
    )


def decode_capture(rx_bytes: bytes):
    """Reconstruct the full 182-bit packet (known sync prefix + captured
    payload) and run it through proflame2_protocol's decode/verify gate.
    This is the FIXED-OFFSET version, used with hardware sync detection
    (--listen-seconds mode) where the CC1101's own correlator already found
    the alignment for us. See software_sync_search() below for the
    alternative that doesn't depend on hardware sync detection at all."""
    captured_bits = bytes_to_bits(rx_bytes)
    full_raw = (SYNC_RAW_BITS + captured_bits)[:182]
    symbols = raw_bits_to_symbols(full_raw)
    return decode_packet_symbols(symbols, CHECKSUM)


BIT_PERIOD_US = 411.206  # Empirically calibrated from real pigpio-captured data
                          # (weighted least-squares over ~1300 known 1-bit/2-bit
                          # runs) - theoretical 1e6/2400=416.667us was measurably
                          # off; only used as a bootstrap starting point for
                          # two_pass_locally_calibrated_search()'s local
                          # recalibration, but a more accurate starting point
                          # catches more real candidates in the cheap pre-filter.
PACKET_RAW_BITS = 182       # 7 words x 13 symbols x 2 raw bits/symbol


def filter_glitches(runs, min_duration_us=None):
    """Merge out implausibly short runs before quantization. A real
    Manchester chip-bit can't be shorter than one bit period (416.7us) -
    anything meaningfully under that (comparator glitch, brief noise blip)
    isn't a real transition. Forcing every run to count as >=1 bit
    (quantize_runs_to_bits' max(1, ...) floor) turns each of these into a
    phantom bit flip that was never really there, corrupting alignment for
    any packet window that contains one.

    Merging removes the short run and combines its two neighbors (which
    are necessarily the same level, since runs always alternate) into one
    continuous run - i.e. "this glitch didn't happen, the level was
    actually constant across it."

    Default threshold is 1/3 of a bit period (~139us) - comfortably below
    any real minimum (416.7us) with margin, while still well above typical
    single-sample measurement noise at our achieved polling rate.
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


def quantize_runs_to_bits(runs):
    """Convert a run-length capture [(level, duration_us), ...] into a raw
    bit string.

    DRIFT-CORRECTED quantization: rather than rounding each run's duration
    to the nearest bit count independently (which lets small per-run timing
    errors compound - a single borderline run, e.g. ~630us sitting right on
    the boundary between 1 and 2 bit-periods, can misjudge by one bit and
    shift every subsequent bit in that packet, cascading into a parity/
    checksum failure even when the overall timing looks clean), this tracks
    CUMULATIVE elapsed time and derives each run's bit count from where
    that puts us on the absolute timeline. A small error in one run gets
    self-corrected on the next rather than accumulating - the same
    principle a real hardware bit-clock-recovery circuit uses, just done
    here in software against already-captured samples."""
    bits = []
    cum_time_us = 0.0
    cum_bits = 0
    for level, dur_us in runs:
        cum_time_us += dur_us
        target_bits = round(cum_time_us / BIT_PERIOD_US)
        n = max(1, target_bits - cum_bits)
        cum_bits += n
        bits.append(str(level) * n)
    return "".join(bits)


def software_sync_search(raw_bits: str, sync_bits: str = SYNC_RAW_BITS,
                          checksum: ChecksumConstants = None):
    """Slide a window across raw_bits looking for every occurrence of
    sync_bits (exact string match - no tolerance for a 1-bit-off match,
    same strictness as CC1101's own 16/16 hardware mode would give). On
    each hit, try decoding the following bits as one full 182-bit packet.

    This is the software-side alternative to hardware sync-word detection -
    doesn't depend on CC1101's own bit-clock-recovery/correlator locking
    onto a preamble this protocol doesn't really have. Works directly on
    the same quantized bit string quantize_runs_to_bits() produces from a
    --raw-capture run, so it can be tested against data already proven
    clean instead of needing new hardware captures for every iteration.

    Returns a list of (bit_offset, DecodedBurst) for every sync match
    found, whether or not that particular match decoded validly - the
    caller can filter on .valid to see genuine packets vs coincidental
    sync-pattern matches in noise/other data.
    """
    if checksum is None:
        checksum = CHECKSUM
    results = []
    search_from = 0
    while True:
        idx = raw_bits.find(sync_bits, search_from)
        if idx == -1:
            break
        candidate = raw_bits[idx: idx + PACKET_RAW_BITS]
        if len(candidate) == PACKET_RAW_BITS:
            try:
                symbols = raw_bits_to_symbols(candidate)
                decoded = decode_packet_symbols(symbols, checksum)
                results.append((idx, decoded))
            except ValueError:
                pass  # not a valid Manchester pair at this offset - not a real match
        search_from = idx + 1  # advance by 1 bit, not len(sync_bits) - repeats
                                 # in the burst could overlap-adjacent in edge cases
    return results


def software_sync_search_multiphase(runs, sync_bits: str = SYNC_RAW_BITS,
                                     checksum: ChecksumConstants = None,
                                     max_bits: int = PACKET_RAW_BITS,
                                     max_sync_errors: int = 2):
    """Like software_sync_search(), but doesn't trust a single global phase
    reference. Instead tries EVERY run boundary in the raw capture as a
    candidate phase-zero, re-running drift-corrected quantization fresh
    from each one.

    Why this matters: quantize_runs_to_bits() anchors its cumulative-time
    drift correction starting from the very first run. If the capture's
    leading segment isn't actually a coherent 416.7us-periodic clock (e.g.
    it's AGC still settling, not real Manchester data), that assumption is
    wrong from the start, and everything downstream inherits a phase error
    even with drift correction - drift correction only prevents RANDOM
    jitter from compounding, it doesn't fix a wrong starting phase.

    FUZZY sync matching: rather than requiring a bit-perfect match against
    all 16 sync bits (which a single quantization error anywhere in that
    window breaks completely, no partial credit), this accepts a candidate
    if it's within max_sync_errors bit-flips (Hamming distance) of
    sync_bits - similar in spirit to CC1101's own "15/16" hardware mode,
    but more permissive since we're not limited to hardware register
    granularity. This is safe to loosen because decode_packet_symbols()
    downstream is the real gatekeeper: a false sync match still has to pass
    guard/parity checks on all 7 words AND two independent checksums to be
    accepted as .valid - an extremely strong filter that a random fuzzy
    sync collision is very unlikely to pass.

    This is slower (O(n_runs) candidate phases x up to max_bits runs each)
    but far more robust to real captures where we don't know in advance
    where a genuine, cleanly-clocked segment begins - which is exactly the
    problem a real preamble exists to solve, and exactly what we're missing
    here.

    Returns a list of (run_index, DecodedBurst) for every candidate phase
    that produced a sync match, whether or not it decoded validly.
    """
    if checksum is None:
        checksum = CHECKSUM
    results = []
    n_sync = len(sync_bits)

    for start_idx in range(len(runs)):
        bits = []
        cum_time_us = 0.0
        cum_bits = 0
        for level, dur_us in runs[start_idx:]:
            cum_time_us += dur_us
            target_bits = round(cum_time_us / BIT_PERIOD_US)
            n = max(1, target_bits - cum_bits)
            cum_bits += n
            bits.append(str(level) * n)
            if cum_bits >= max_bits:
                break
        candidate_bits = "".join(bits)

        if len(candidate_bits) < n_sync:
            continue
        prefix = candidate_bits[:n_sync]
        hamming = sum(1 for a, b in zip(prefix, sync_bits) if a != b)
        if hamming > max_sync_errors:
            continue

        full = candidate_bits[:PACKET_RAW_BITS]
        if len(full) != PACKET_RAW_BITS:
            continue
        decoded = decode_with_bitflip_correction(full, checksum)
        if decoded is not None:
            results.append((start_idx, decoded))

    return results


def quantize_window(window, period, max_bits=PACKET_RAW_BITS):
    """Drift-corrected quantization of a run window at a given bit period.
    Shared helper used by both the global-period search and the locally-
    calibrated two-pass search below."""
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
    """The search that actually works on real hardware data, found via
    direct debugging against real captures - see module history.

    Root cause this solves: a single GLOBAL bit-period constant, even
    calibrated from pooled real data, isn't accurate enough for any single
    specific packet occurrence - real oscillator/measurement conditions
    vary capture to capture (and likely burst to burst) by enough to matter
    over a 182-bit window, even when using genuinely clean pigpio hardware-
    timestamped data (busy-loop data had a worse, different problem -
    missing pulses from OS scheduler preemption - fixed separately by
    switching to pigpio; this fixes a distinct, subtler timing-precision
    issue that persisted even after that).

    Two passes per candidate phase:
      1. Quantize just the first 2 words (52 bits) using global_period,
         confirm they're structurally valid (sync/guard/parity - NOT
         checksums yet, those need the full packet). This is cheap and
         filters out non-candidates fast.
      2. Calibrate a LOCAL period from exactly how much real elapsed time
         those confirmed-good 52 bits actually spanned, then re-quantize
         the ENTIRE 182-bit packet using that refined, per-occurrence
         period instead of the global constant.

    This works because the first 2 words being structurally valid under
    the global period means the phase alignment (WHERE bit boundaries
    start) is right - the remaining error is a small, consistent per-bit
    timing offset specific to this exact capture, which local calibration
    corrects before it can compound across the rest of the packet.

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

        # Fast pre-filter: fuzzy sync check at the global period before
        # doing the more expensive two-pass work.
        quick = quantize_window(window, global_period, max_bits=n_sync)
        if len(quick) < n_sync:
            continue
        hamming = sum(1 for a, b in zip(quick[:n_sync], sync_bits) if a != b)
        if hamming > max_sync_errors:
            continue

        # Pass 1: confirm first 2 words are structurally valid under the
        # global period (sync/guard/parity only - cheap, no checksum yet).
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

        # Calibrate local period from exactly how much real elapsed time
        # those confirmed-good 52 bits spanned.
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

        # Pass 2: re-quantize the WHOLE packet with the locally-calibrated
        # period and attempt a full decode (checksums now checked for real).
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


def decode_with_bitflip_correction(raw_bits_182: str, checksum: ChecksumConstants,
                                    max_flips: int = 1):
    """Try decoding a 182-bit candidate as-is; if that fails, try every
    possible single-bit flip (or up to max_flips bits) and accept the
    first correction that produces a fully valid decode.

    Why this is safe rather than reckless: a valid decode requires guard
    bits AND odd parity on all 7 words AND two independent checksums to
    all agree simultaneously. That's an extremely narrow target - a wrong
    bit-flip "fixing" one word almost never also happens to satisfy every
    other check by coincidence. In practice there's essentially only one
    correction (if any) that makes everything agree, which is why this is
    a correction, not a guess.

    Motivated by real hardware data: a captured packet was found to be a
    single-symbol (single-bit) error away from a perfect decode, with
    every other bit in a 182-bit window exactly correct - i.e. real
    capture quantization is far more accurate than repeatedly failing to
    decode anything suggested; the decoder just had zero tolerance for
    that one remaining bit.

    Returns the DecodedBurst if a valid decode was found (original or
    corrected), else None. Only 1-bit correction is attempted by default -
    trying 2-bit combinations is combinatorially worse (182 choose 2 =
    ~16,000 attempts) and hasn't been shown necessary yet.
    """
    try:
        symbols = raw_bits_to_symbols(raw_bits_182)
        decoded = decode_packet_symbols(symbols, checksum)
        if decoded.valid:
            return decoded
    except ValueError:
        pass

    if max_flips >= 1:
        bits = list(raw_bits_182)
        for i in range(len(bits)):
            original = bits[i]
            bits[i] = "1" if original == "0" else "0"
            flipped = "".join(bits)
            bits[i] = original  # restore for next iteration
            try:
                symbols = raw_bits_to_symbols(flipped)
                decoded = decode_packet_symbols(symbols, checksum)
                if decoded.valid:
                    return decoded
            except ValueError:
                continue

    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-seconds", type=int, default=30,
                         help="How long to listen for packets before exiting (default 30)")
    parser.add_argument("--rssi-scan", action="store_true",
                         help="Diagnostic mode: continuously print RSSI, bypassing sync-word "
                              "matching entirely. Use this first if packet capture gets 0/0.")
    parser.add_argument("--gdo0-watch", action="store_true",
                         help="Diagnostic mode: hardware-interrupt watch for GDO0 rising edges "
                              "(sync-word detect), independent of packet-complete timing logic.")
    parser.add_argument("--raw-capture", action="store_true",
                         help="Diagnostic mode: bypass the packet engine entirely, capture the "
                              "raw demodulated bitstream (RSSI-triggered), print run-length "
                              "encoded output for ground-truth comparison against our sync word.")
    parser.add_argument("--max-bursts", type=int, default=50,
                         help="Max triggered bursts to capture in --raw-capture before stopping "
                              "early, even if --listen-seconds hasn't elapsed (default 50)")
    parser.add_argument("--trigger-dbm", type=float, default=-85.0,
                         help="RSSI threshold for --raw-capture to trigger a capture (default "
                              "-85). Noise floor sits around -94 to -99dBm; real presses have "
                              "measured -39 to -85dBm depending on distance - raise toward -95 "
                              "only if genuine presses are being missed, since that risks false "
                              "triggers on noise (single 'stuck' runs with no real data).")
    parser.add_argument("--loop-bench", action="store_true",
                         help="Diagnostic mode: benchmark this Pi's achievable GPIO.input() "
                              "polling rate. Run this if raw-capture pulse widths look "
                              "suspiciously uniform - tests whether the loop itself, not the "
                              "signal, is the limiting factor.")
    parser.add_argument("--save-runs", type=str, default=None,
                         help="With --raw-capture or --pigpio-capture: save the COMPLETE run "
                              "list for every burst to '{prefix}_burstN.json' (terminal only "
                              "ever shows the first 60 of what's often 600+ runs).")
    parser.add_argument("--pigpio-capture", action="store_true",
                         help="Like --raw-capture, but uses pigpio's hardware-timestamped edge "
                              "callbacks instead of a Python busy-loop - fixes multi-millisecond "
                              "scheduler-preemption gaps confirmed corrupting busy-loop captures. "
                              "Requires pigpiod running (sudo apt install pigpio python3-pigpio; "
                              "sudo systemctl start pigpiod).")
    args = parser.parse_args()

    radio = CC1101RX()
    attempts = 0
    hits = 0

    try:
        print("Powering on radio...")
        if not radio.power_on():
            print("PWRGD never went high - LDO/power sequencing failure. Aborting.")
            sys.exit(1)

        if args.loop_bench:
            radio.loop_bench(seconds=min(args.listen_seconds, 5))
            return

        if args.pigpio_capture:
            radio.configure_async_sniffer()
            print("Configured async sniffer mode (packet engine bypassed)")
            radio.dump_key_registers()
            radio.raw_capture_pigpio(listen_seconds=args.listen_seconds, max_bursts=args.max_bursts,
                                      trigger_dbm=args.trigger_dbm, save_runs_prefix=args.save_runs)
            return

        if args.raw_capture:
            radio.configure_async_sniffer()
            print("Configured async sniffer mode (packet engine bypassed)")
            radio.dump_key_registers()
            radio.raw_capture(listen_seconds=args.listen_seconds, max_bursts=args.max_bursts,
                               trigger_dbm=args.trigger_dbm, save_runs_prefix=args.save_runs)
            return

        info = radio.configure_ook_rx()
        print(f"Configured: {info['actual_freq_hz']/1e6:.4f} MHz, "
              f"sync={info['sync']}, PKTLEN={info['pktlen_bytes']} bytes")
        radio.dump_key_registers()

        if args.rssi_scan:
            radio.rssi_scan(seconds=args.listen_seconds)
            return

        if args.gdo0_watch:
            radio.gdo0_watch(seconds=args.listen_seconds)
            return

        print(f"Listening for {args.listen_seconds}s - press the remote now...\n")

        deadline = time.time() + args.listen_seconds
        while time.time() < deadline:
            radio.arm_rx()
            data = radio.wait_for_packet(timeout_s=1.0)
            if data is None:
                continue

            attempts += 1
            print(f"[{attempts}] Sync detected, captured {len(data)} bytes: {data.hex()}")

            decoded = decode_capture(data)
            if decoded.valid:
                hits += 1
                state = command_bytes_to_state(decoded.command1, decoded.command2)
                print(f"    VALID  serial=0x{decoded.serial_number:06X}")
                print(f"      {format_state(state)}")
            else:
                print(f"    INVALID  errors={decoded.errors}")

        print(f"\nDone. {hits}/{attempts} captures decoded valid.")
        if attempts == 0:
            print("GDO0 never asserted. RSSI already confirmed RF reception works - "
                  "try --raw-capture to see the actual demodulated bitstream and "
                  "compare it directly against the computed sync word.")

    finally:
        radio.power_off()
        radio.close()


if __name__ == "__main__":
    main()