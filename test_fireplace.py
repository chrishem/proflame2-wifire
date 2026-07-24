#!/usr/bin/env python3
"""
Send a Proflame2 command to the fireplace, with NeoPixel status indication
and LDO power sequencing via a shared set_radio_ldo() helper.

Usage:
  sudo ./test_fireplace.py --power on --flame 3 --fan 2 --light 1 --aux
  sudo ./test_fireplace.py --power off
  ./test_fireplace.py --power on --flame 1   (auto re-execs under sudo)
  ./test_fireplace.py --power on --flame 1 --fast   (batch mode: quiet + faster)

Exit codes:
  0 = success
  1 = radio LDO power-on failed (PWRGD never went high)
  2 = CC1101 configure/transmit failure

Always prints one final machine-parseable line regardless of --quiet:
  RESULT: power=on flame=3 fan=2 light=0 aux=off status=ok
  RESULT: power=on flame=3 fan=2 light=0 aux=off status=error:<message>

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

QUIET = False       # set from --quiet in main()
POST_DELAY_S = 1.0  # set from --fast in main()


def log(msg):
    """Print unless --quiet was given. Errors and the final RESULT line
    bypass this and always print."""
    if not QUIET:
        print(msg)


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
    parser = argparse.ArgumentParser(description="Send a Proflame2 command to the fireplace.")
    parser.add_argument("--power", choices=["on", "off"], required=True,
                         help="Fireplace power state to request.")
    parser.add_argument("--flame", type=int, default=None, choices=range(0, 7), metavar="0-6",
                         help="Flame height (0-6). Must be 1-6 when --power on - 0 "
                              "means 'no target flame', a correctly-executed no-op "
                              "that looks like the fireplace ignoring the command. "
                              "Defaults to 0 for --power off, where it's moot.")
    parser.add_argument("--fan", type=int, default=0, choices=range(0, 7), metavar="0-6",
                         help="Fan speed (0-6).")
    parser.add_argument("--light", type=int, default=0, choices=range(0, 7), metavar="0-6",
                         help="Light level (0-6).")
    parser.add_argument("--aux", action="store_true",
                         help="Enable the auxiliary/secondary burner.")
    parser.add_argument("--fast", action="store_true",
                         help="Batch mode: shrink cosmetic post-action delays and "
                              "suppress step-by-step output (only the final RESULT "
                              "line and any errors print).")
    args = parser.parse_args()

    if args.power == "on" and (args.flame is None or args.flame == 0):
        parser.error("--flame must be 1-6 when --power on (0 means 'no target "
                      "flame' - a correctly-executed no-op that looks like the "
                      "fireplace ignoring the command)")
    if args.flame is None:
        args.flame = 0  # fine for --power off; visible flame level is moot

    return args


def result_line(args, status):
    aux_desc = "on" if args.aux else "off"
    return (f"RESULT: power={args.power} flame={args.flame} fan={args.fan} "
            f"light={args.light} aux={aux_desc} status={status}")


def main():
    global QUIET, POST_DELAY_S
    args = parse_args()
    QUIET = args.fast
    POST_DELAY_S = 0.2 if args.fast else 1.0

    if os.geteuid() != 0:
        log("Not running as root - re-executing under sudo...")
        os.execvp("sudo", ["sudo", sys.executable, os.path.abspath(__file__)] + sys.argv[1:])

    strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    strip.begin()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RAD_EN, GPIO.OUT)
    GPIO.setup(PWRGD, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    radio = None

    try:
        # Trusting these are already off from prior cleanup - no delay needed here.
        log("NeoPixel off, LDO off")
        set_pixel(strip, COLOR_OFF)
        GPIO.output(RAD_EN, GPIO.LOW)

        log("Powering on...")
        set_radio_ldo(strip, turn_on=True)  # exits(1) internally on PWRGD failure

        log(f"Building and sending CC1101 command: power={args.power} "
            f"flame={args.flame} fan={args.fan} light={args.light} aux={args.aux}")
        # pilot_cpi is a SEPARATE, deliberately user-controlled setting (CPI keeps
        # the pilot lit - faster ignition, wards off condensation/rust in winter;
        # IPI shuts the pilot off). It must NOT be forced by a power command, and
        # isn't exposed as a CLI flag here - hardcoded to False (IPI) until there's
        # a deliberate reason to change it.
        state = FireplaceState(
            power=(args.power == "on"),
            pilot_cpi=False,
            flame=args.flame,
            fan=args.fan,
            light=args.light,
            aux=args.aux,
        )
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
            print(result_line(args, f"error:{e}"))
            sys.exit(2)

        set_pixel(strip, COLOR_OFF)
        time.sleep(POST_DELAY_S)

        log("Powering off...")
        set_radio_ldo(strip, turn_on=False)

        time.sleep(POST_DELAY_S)

        set_pixel(strip, COLOR_OFF)

        print(result_line(args, "ok"))

    finally:
        if radio is not None:
            radio.spi.close()  # avoid double GPIO.cleanup() via radio.close()
        GPIO.output(RAD_EN, GPIO.LOW)
        GPIO.cleanup()


if __name__ == "__main__":
    main()
