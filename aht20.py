#!/usr/bin/env python3
"""
aht20_read.py: Read temperature/humidity from an AHT20 sensor over I2C (e.g. via a Stemma QT connector).
Requires smbus2: pip install smbus2 --break-system-packages
"""

import argparse
import sys
import time

from smbus2 import SMBus, i2c_msg

#AHT20_ADDR = 0x38


def init_sensor(bus: SMBus, addr: int):
    write = i2c_msg.write(addr, [0xBE, 0x08, 0x00])
    try:
        bus.i2c_rdwr(write)
    except OSError:
        print(f"Can't open I2C Device at {addr:#x}; is Address correct?")
        sys.exit(1)
    time.sleep(0.02)


def read_sensor(bus: SMBus, addr: int):
    trigger = i2c_msg.write(addr, [0xAC, 0x33, 0x00])
    bus.i2c_rdwr(trigger)
    time.sleep(0.08)  # datasheet: wait >=75ms for conversion

    read = i2c_msg.read(addr, 6)
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
    parser.add_argument("--interval", type=float, default=0, help="Repeat every N seconds (default: read once and exit)")
    parser.add_argument("--address", type=lambda x: int(x,0), default=0x38, help="I2C Address (eg 0x38 or 56)")
    args = parser.parse_args()

    try:
        bus = SMBus(args.bus)
    except FileNotFoundError:
        print(f"Can't open /dev/i2c-{args.bus} - is I2C enabled (raspi-config)? "
              f"Is smbus2 installed?")
        sys.exit(1)

    try:
        init_sensor(bus, args.address)
        while True:
            try:
                temp_c, temp_f, humidity = read_sensor(bus, args.address)
                print(f"Temp: {temp_f:.2f}F   Humidity: {humidity:.2f}%")
            except RuntimeError as e:
                print(f"Read failed: {e}")

            if args.interval <= 0:
                break
            time.sleep(args.interval)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
