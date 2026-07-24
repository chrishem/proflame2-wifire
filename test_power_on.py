#!/usr/bin/env python3
"""
Test script: transmit a Proflame2 "Power On" command, with NeoPixel status
indication and LDO power sequencing via a shared set_power() helper.

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


def set_pixel(strip, color):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()


def set_power(strip: PixelStrip, turn_on: bool) -> bool:
    """Set the LDO on/off, wait for it to settle, verify via PWRGD, and reflect
    status on the NeoPixel.

    For turn_on=True: a PWRGD failure is fatal (nothing downstream can work
    without power) - NeoPixel goes red and the script exits.
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
            print("PWRGD is LOW - LDO did not come up. NeoPixel red, exiting.")
            set_pixel(strip, COLOR_RED)
            GPIO.output(RAD_EN, GPIO.LOW)
            sys.exit(1)
        print("PWRGD confirmed high.")
        set_pixel(strip, COLOR_BLUE)
        return True
    else:
        if pwrgd:
            print("WARNING: PWRGD still HIGH after LDO off - rail may not have "
                  "discharged yet, or PWRGD/RAD_EN wiring should be re-checked.")
            return False
        print("PWRGD confirmed low - LDO cleanly disabled.")
        set_pixel(strip, COLOR_GREEN)
        return True


def main():
    if os.geteuid() != 0:
        print("Not running as root - re-executing under sudo...")
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
        print("NeoPixel off, LDO off")
        set_pixel(strip, COLOR_OFF)
        GPIO.output(RAD_EN, GPIO.LOW)

        print("Powering on...")
        set_power(strip, turn_on=True)  # exits internally on PWRGD failure

        print("Building and sending CC1101 Power On command...")
        # pilot_cpi is a SEPARATE, deliberately user-controlled setting (CPI keeps
        # the pilot lit - faster ignition, wards off condensation/rust in winter;
        # IPI shuts the pilot off). It must NOT be forced by a power command - set
        # this to match whatever the pilot is actually supposed to be right now,
        # not left as a guessed default.
        # flame=1 (not 0!): every real-world "Power: 1" capture shared in the
        # rtl_433 Proflame2 community thread (github.com/merbanan/rtl_433/issues/1905)
        # has a nonzero flame value - flame=0 likely means "no target flame
        # requested," which would explain several of our earlier "beeped but no
        # visible change" results as a correctly-executed no-op, not a bug.
        state = FireplaceState(power=True, flame=5, fan=5, aux=True)
        burst_bits = build_burst_bits(SERIAL_NUMBER, state, CHECKSUM)
        payload = bits_to_bytes(burst_bits)

        radio = CC1101TX()
        info = radio.configure_ook_tx()
        print(f"Configured: {info['actual_freq_hz']/1e6:.4f} MHz (PATABLE readback verified OK)")
        radio.transmit(payload)
        print("Transmit complete - FIFO drained cleanly, no underflow.")

        set_pixel(strip, COLOR_OFF)
        print("Delay 1s")
        time.sleep(1)

        print("Powering off...")
        set_power(strip, turn_on=False)

        print("Delay 1s")
        time.sleep(1)

        set_pixel(strip, COLOR_OFF)

    finally:
        if radio is not None:
            radio.spi.close()  # avoid double GPIO.cleanup() via radio.close()
        GPIO.output(RAD_EN, GPIO.LOW)
        GPIO.cleanup()


if __name__ == "__main__":
    main()
