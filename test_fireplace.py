#!/usr/bin/env python3
"""
Send a Proflame2 command to the fireplace, tracking persisted state between
runs so unspecified fields carry forward instead of resetting to 0/False.

The real remote always transmits its FULL current state, not deltas - so we
have to do the same. Any field you don't pass on the command line is read
from the last-known state (fireplace_state.json, next to this script) rather
than defaulting to 0/off, which would otherwise silently stomp on settings
you didn't intend to change.

Usage:
  sudo ./test_fireplace.py --power on --flame 3 --fan 2 --light 1 --backburner on
  sudo ./test_fireplace.py --power on --backburner on   (carries over flame/fan/
                                                          light/pilot_cpi from
                                                          last run)
  sudo ./test_fireplace.py --power off
  ./test_fireplace.py --power on --flame 1 --fast   (batch mode: quiet + faster)
  ./test_fireplace.py --status   (no root needed - just prints last-known state)

First run ever (no state file yet): all unspecified fields default to a
conservative 0/off baseline, and the script prints a clear warning that this
is an ASSUMED starting point, not a confirmed reading - there's no way to
electronically verify the fireplace's real current state.

Exit codes:
  0 = success
  1 = radio LDO power-on failed (PWRGD never went high)
  2 = CC1101 configure/transmit failure

Always prints one final machine-parseable line regardless of --fast:
  RESULT: power=on flame=3 fan=2 light=0 backburner=off pilot=ipi status=ok
  RESULT: power=on flame=3 fan=2 light=0 backburner=off pilot=ipi status=error:<message>

Pin map (per WiFirePi interposer board):
  RAD_EN (LDO enable, "LDOCTRL")  GPIO27
  PWRGD  (LDO power-good)          GPIO22
  NeoPixel data                    GPIO18
  CC1101 GDO-0                     GPIO5   (not used by this script - reserved
                                             for future RX/status monitoring)
  CC1101 GDO-2                     GPIO6   (not used by this script - reserved)
  CC1101 CHIPSEL                   GPIO8   (hardware SPI0 CE0 - handled
                                             automatically by spidev.open(0, 0))
  Pi power button                  GPIO17  (not used by this script)
  Misc spare                       GPIO23  (not used by this script)

Requires root (rpi_ws281x needs /dev/mem access for DMA/PWM) - the script
re-execs itself under sudo automatically if not already root.
"""

import argparse
import json
import os
import sys
import time

import RPi.GPIO as GPIO
from rpi_ws281x import PixelStrip, Color

from proflame2_protocol import FireplaceState, ChecksumConstants, build_burst_bits, bits_to_bytes
from cc1101_tx import CC1101TX

RAD_EN = 27
PWRGD = 22

LED_PIN = 18
LED_COUNT = 1
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 80
LED_INVERT = False
LED_CHANNEL = 0

COLOR_OFF = Color(0, 0, 0)
COLOR_RED = Color(255, 0, 0)
COLOR_YELLOW = Color(255, 180, 0)
COLOR_BLUE = Color(0, 0, 255)
COLOR_GREEN = Color(0, 255, 0)

SERIAL_NUMBER = 0xA3D502
CHECKSUM = ChecksumConstants(c1=0x7, d1=0x5, c2=0x4, d2=0xD)

LDO_SETTLE_S = 0.25  # MCP1727 startup is well under this; generous margin

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fireplace_state.json")

# Conservative assumed baseline ONLY used if no state file exists yet.
# This is a guess, not a confirmed reading - there's no way to electronically
# verify the fireplace's actual current state.
DEFAULT_STATE = {
    "power": False,
    "pilot_cpi": False,
    "flame": 0,
    "fan": 0,
    "light": 0,
    "front": False,
}

QUIET = False       # set from --fast in main()
POST_DELAY_S = 1.0  # set from --fast in main()


def log(msg):
    """Print unless --fast was given. Errors and the final RESULT line
    bypass this and always print."""
    if not QUIET:
        print(msg)


def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE) as f:
            return json.load(f)
    log(f"No state file found at {STATE_FILE} - this is the first run, or it "
        f"was deleted. Assuming baseline (power off, everything 0/off). This "
        f"is a GUESS, not a confirmed reading - verify against the real "
        f"fireplace if this matters.")
    return dict(DEFAULT_STATE)


def save_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)


def set_pixel(strip, color):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()


def set_radio_ldo(strip: PixelStrip, turn_on: bool) -> bool:
    """Set the LDO on/off, wait for it to settle, verify via PWRGD, and reflect
    status on the NeoPixel.

    For turn_on=True: a PWRGD failure is fatal (nothing downstream can work
    without power) - NeoPixel goes red and the script exits with code 1.
    For turn_on=False: a PWRGD mismatch is a warning, not fatal - there's
    nothing left to protect by aborting during cleanup, so it just returns
    False and lets the caller continue.
    """
    if turn_on:
        set_pixel(strip, COLOR_YELLOW)
        GPIO.output(RAD_EN, GPIO.HIGH)
    else:
        GPIO.output(RAD_EN, GPIO.LOW)

    time.sleep(LDO_SETTLE_S)
    pwrgd = GPIO.input(PWRGD)

    if turn_on:
        if not pwrgd:
            print("ERROR: PWRGD is LOW - LDO did not come up.", file=sys.stderr)
            set_pixel(strip, COLOR_RED)
            GPIO.output(RAD_EN, GPIO.LOW)
            sys.exit(1)
        log("PWRGD confirmed high.")
        set_pixel(strip, COLOR_BLUE)
        return True
    else:
        if pwrgd:
            log("WARNING: PWRGD still HIGH after LDO off - rail may not have "
                "discharged yet, or PWRGD/RAD_EN wiring should be re-checked.")
            return False
        log("PWRGD confirmed low - LDO cleanly disabled.")
        set_pixel(strip, COLOR_GREEN)
        return True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a Proflame2 command to the fireplace. Any field not "
                     "specified carries forward from the last run's state.")
    parser.add_argument("--power", choices=["on", "off"], default=None,
                         help="Fireplace power state to request. Required unless --status.")
    parser.add_argument("--flame", type=int, default=None, choices=range(0, 7), metavar="0-6",
                         help="Flame height (0-6). Must be 1-6 (explicitly or via "
                              "carried-over state) when --power on - 0 means 'no "
                              "target flame', a correctly-executed no-op. If "
                              "omitted, carries forward from last run.")
    parser.add_argument("--fan", type=int, default=None, choices=range(0, 7), metavar="0-6",
                         help="Fan speed (0-6). If omitted, carries forward from last run.")
    parser.add_argument("--light", type=int, default=None, choices=range(0, 7), metavar="0-6",
                         help="Light level (0-6). If omitted, carries forward from last run.")
    parser.add_argument("--backburner", choices=["on", "off"], default=None,
                         help="Rear/secondary burner (library-internally this is the "
                              "protocol's 'front' bit - confirmed 2026-07-24 that on "
                              "this unit it controls the rear burner, not anything "
                              "front-facing). If omitted, carries forward from last run.")
    parser.add_argument("--pilot-cpi", choices=["cpi", "ipi"], default=None,
                         help="Pilot mode: cpi=continuous pilot lit, ipi=pilot off. "
                              "This is a deliberate, separate seasonal setting - "
                              "NOT tied to power on/off. If omitted, carries "
                              "forward from last run (never silently changed).")
    parser.add_argument("--status", action="store_true",
                         help="Print the last-known fireplace state and exit - no "
                              "radio, no LDO, no root needed. This reads the "
                              "persisted state file, which reflects what we last "
                              "COMMANDED, not a live/confirmed reading (no receiver "
                              "hardware exists to verify the fireplace's real state).")
    parser.add_argument("--fast", action="store_true",
                         help="Batch mode: shrink cosmetic post-action delays and "
                              "suppress step-by-step output (only the final RESULT "
                              "line and any errors print).")
    args = parser.parse_args()

    if args.status:
        return args  # --power not required/validated in status mode

    if args.power is None:
        parser.error("--power is required unless --status is given")

    return args


def merge_state(persisted, args):
    """Overlay explicitly-provided CLI args onto the persisted state. Fields
    not passed on the command line keep their last-known value."""
    merged = dict(persisted)
    merged.pop("aux", None)  # retired - confirmed no observable effect on this unit
    merged["power"] = (args.power == "on")
    if args.flame is not None:
        merged["flame"] = args.flame
    if args.fan is not None:
        merged["fan"] = args.fan
    if args.light is not None:
        merged["light"] = args.light
    if args.backburner is not None:
        # "backburner" is the user-facing name; the library field is still "front"
        # (confirmed 2026-07-24 that's the bit that actually drives this on this unit)
        merged["front"] = (args.backburner == "on")
    if args.pilot_cpi is not None:
        merged["pilot_cpi"] = (args.pilot_cpi == "cpi")
    return merged


def result_line(state, status):
    return (f"RESULT: power={'on' if state['power'] else 'off'} "
            f"flame={state['flame']} fan={state['fan']} light={state['light']} "
            f"backburner={'on' if state['front'] else 'off'} "
            f"pilot={'cpi' if state['pilot_cpi'] else 'ipi'} status={status}")


def render_range(value, max_value=6):
    """Render a 0-max_value setting as a visual range, e.g. '0 1 [2] 3 4 5 6'.
    Doubles as a preview of what a small physical display could show later."""
    digits = [f"[{i}]" if i == value else str(i) for i in range(max_value + 1)]
    return f"{value} of {max_value}   " + " ".join(digits)


def show_status():
    persisted = load_state()
    print("Last known fireplace state (from persisted state file - this reflects "
          "what we last COMMANDED, NOT a live/confirmed reading - no receiver "
          "hardware exists to verify this against the real fireplace):")
    print(f"  power:      {'on' if persisted.get('power') else 'off'}")
    print(f"  flame:      {render_range(persisted.get('flame', 0))}")
    print(f"  fan:        {render_range(persisted.get('fan', 0))}")
    print(f"  light:      {render_range(persisted.get('light', 0))}")
    print(f"  backburner: {'on' if persisted.get('front') else 'off'}")
    print(f"  pilot:      {'cpi' if persisted.get('pilot_cpi') else 'ipi'}")
    if "aux" in persisted:
        print("  (stale 'aux' field present in state file - no longer used, "
              "will be dropped automatically on next command)")


def main():
    global QUIET, POST_DELAY_S
    args = parse_args()

    if args.status:
        show_status()
        return

    QUIET = args.fast
    POST_DELAY_S = 0.2 if args.fast else 1.0

    if os.geteuid() != 0:
        log("Not running as root - re-executing under sudo...")
        os.execvp("sudo", ["sudo", sys.executable, os.path.abspath(__file__)] + sys.argv[1:])

    persisted = load_state()
    merged = merge_state(persisted, args)

    if merged["power"] and merged["flame"] == 0:
        print("ERROR: resulting flame=0 with power=on (0 means 'no target flame', "
              "a correctly-executed no-op). Pass --flame 1-6, or check the "
              "carried-over state in " + STATE_FILE, file=sys.stderr)
        sys.exit(2)

    state = FireplaceState(
        power=merged["power"],
        pilot_cpi=merged["pilot_cpi"],
        flame=merged["flame"],
        fan=merged["fan"],
        light=merged["light"],
        front=merged["front"],
    )

    strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    strip.begin()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RAD_EN, GPIO.OUT)
    GPIO.setup(PWRGD, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    radio = None

    try:
        log("NeoPixel off, LDO off")
        set_pixel(strip, COLOR_OFF)
        GPIO.output(RAD_EN, GPIO.LOW)

        log("Powering on...")
        set_radio_ldo(strip, turn_on=True)  # exits(1) internally on PWRGD failure

        log(f"Building and sending CC1101 command (carried-over fields marked "
            f"with last-known value where not passed on CLI): {merged}")

        burst_bits = build_burst_bits(SERIAL_NUMBER, state, CHECKSUM)
        payload = bits_to_bytes(burst_bits)

        try:
            radio = CC1101TX()
            info = radio.configure_ook_tx()
            log(f"Configured: {info['actual_freq_hz']/1e6:.4f} MHz (PATABLE readback verified OK)")
            radio.transmit(payload)
            log("Transmit complete - FIFO drained cleanly, no underflow.")
        except (RuntimeError, ValueError) as e:
            print(f"ERROR: CC1101 transmit failed: {e}", file=sys.stderr)
            set_pixel(strip, COLOR_RED)
            print(result_line(merged, f"error:{e}"))
            sys.exit(2)

        set_pixel(strip, COLOR_OFF)
        time.sleep(POST_DELAY_S)

        log("Powering off...")
        set_radio_ldo(strip, turn_on=False)

        time.sleep(POST_DELAY_S)

        set_pixel(strip, COLOR_OFF)

        save_state(merged)
        log(f"State saved to {STATE_FILE}")

        print(result_line(merged, "ok"))

    finally:
        if radio is not None:
            radio.spi.close()  # avoid double GPIO.cleanup() via radio.close()
        GPIO.output(RAD_EN, GPIO.LOW)
        GPIO.cleanup()


if __name__ == "__main__":
    main()
