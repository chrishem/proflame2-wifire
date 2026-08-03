#!/usr/bin/env python3
import spidev
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(16, GPIO.OUT)
GPIO.output(16, GPIO.HIGH)
time.sleep(0.5)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0

# Reset strobe
spi.xfer2([0x30])
time.sleep(0.1)

partnum = spi.xfer2([0xB0, 0x00])[1]
version = spi.xfer2([0xB1, 0x00])[1]

print(f"PARTNUM: 0x{partnum:02X} (expect 0x00)")
print(f"VERSION: 0x{version:02X} (expect 0x14)")

GPIO.cleanup()
