#!/usr/bin/env python3
"""
aht20_read.py - Read temperature/humidity from an AHT20 sensor over I2C
(e.g. via a Stemma QT connector).

Protocol (AHT20 datasheet):
  - I2C address 0x38
  - Init: write 0xBE, 0x08, 0x00 (only needed once after power-up)
  - Trigger measurement: write 0xAC, 0x33, 0x00
  - Wait ~80ms for conversion
  - Read 6 bytes: [status, hum19:12, hum11:4, hum3:0|temp19:16, temp15:8, temp7:0]
  - humidity_pct = (raw_humidity / 2**20) * 100
  - temp_c = (raw_temp / 2**20) * 200 - 50

Requires smbus2: pip install smbus2 --break-system-packages

Usage:
  python3 aht20_read.py [--bus N] [--interval SECONDS]
"""

import argparse
import sys
import time

from smbus2 import SMBus, i2c_msg

AHT20_ADDR = 0x38


def init_sensor(bus: SMBus):
    write = i2c_msg.write(AHT20_ADDR, [0xBE, 0x08, 0x00])
    bus.i2c_rdwr(write)
    time.sleep(0.02)


def read_sensor(bus: SMBus):
    trigger = i2c_msg.write(AHT20_ADDR, [0xAC, 0x33, 0x00])
    bus.i2c_rdwr(trigger)
    time.sleep(0.08)  # datasheet: wait >=75ms for conversion

    read = i2c_msg.read(AHT20_ADDR, 6)
    bus.i2c_rdwr(read)
    data = list(read)

    status = data[0]
    if status & 0x80:
        raise RuntimeError(f"Sensor still busy (status=0x{status:02X}) - measurement not ready")

    raw_humidity = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
    raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]

    humidity_pct = (raw_humidity / (1 << 20)) * 100
    temp_c = (raw_temp / (1 << 20)) * 200 - 50
    temp_f = temp_c * 9 / 5 + 32

    return temp_c, temp_f, humidity_pct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default 1)")
    parser.add_argument("--interval", type=float, default=0,
                         help="Repeat every N seconds (default: read once and exit)")
    args = parser.parse_args()

    try:
        bus = SMBus(args.bus)
    except FileNotFoundError:
        print(f"Can't open /dev/i2c-{args.bus} - is I2C enabled (raspi-config)? "
              f"Is smbus2 installed?")
        sys.exit(1)

    try:
        init_sensor(bus)
        while True:
            try:
                temp_c, temp_f, humidity = read_sensor(bus)
                print(f"Temp: {temp_c:.2f}C / {temp_f:.2f}F   Humidity: {humidity:.2f}%")
            except RuntimeError as e:
                print(f"Read failed: {e}")

            if args.interval <= 0:
                break
            time.sleep(args.interval)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
