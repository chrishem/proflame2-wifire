"""
CC1101 SPI driver - OOK/ASK transmit only, tuned for the Proflame2 protocol
(314.973 MHz, 2400 baud, pre-Manchester-encoded raw bitstream fed as payload).

Register values and TX approach ported directly from j2deen/proflame2-esp's
proflame2_cc1101.cpp (fixed-length packet mode + manual calibration), not
reconstructed from generic CC1101 examples. This replaces an earlier version
that used infinite-packet-length mode and time-based FIFO pacing, which is
the likely cause of a mid-burst glitch seen in an earlier real transmission
capture (Cpon.sub, 2026-07-21).

Key differences from the earlier version, ported from the verified source:
  - PKTCTRL0 = 0x00 (fixed length, not infinite-length streaming) + PKTLEN
    set to the exact burst size. CC1101 auto-returns to IDLE when done.
  - MCSM0 = 0x04 (manual calibration): explicit SCAL strobe + MARCSTATE
    check before each TX, instead of relying on auto-cal-on-STX timing.
  - FIFO refill is a tight poll loop (check TXBYTES/MARCSTATE, top up
    whatever fits, repeat) with no artificial time.sleep() pacing.
"""

import time
import spidev
import RPi.GPIO as GPIO

RAD_EN = 27
PWRGD = 22

# --- CC1101 strobe/register addresses ---
SRES = 0x30
SCAL = 0x33
STX = 0x35
SIDLE = 0x36
SFRX = 0x3A
SFTX = 0x3B

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
REG_FREND1 = 0x21
REG_FREND0 = 0x22
REG_FSCAL3 = 0x23
REG_FSCAL2 = 0x24
REG_FSCAL1 = 0x25
REG_FSCAL0 = 0x26
REG_TEST2 = 0x2C
REG_TEST1 = 0x2D
REG_TEST0 = 0x2E

MARCSTATE = 0x35   # status register
TXBYTES = 0x3A     # status register
PATABLE = 0x3E
TXFIFO = 0x3F

XOSC_HZ = 26_000_000
TARGET_FREQ_HZ = 314_973_000
TARGET_BAUD = 2400
FIFO_SIZE = 64

# Register table ported verbatim from proflame2-esp/proflame2_cc1101.cpp.
# Frequency registers are computed separately, not part of this table.
CC1101_CONFIG = [
    (REG_IOCFG0, 0x02),
    (REG_FIFOTHR, 0x47),
    # PKTLEN written per-transmission (set to exact burst length)
    (REG_PKTCTRL1, 0x00),
    (REG_PKTCTRL0, 0x00),   # fixed length, CRC off, whitening off, FIFO mode
    (REG_FSCTRL1, 0x06),
    (REG_MDMCFG4, 0xF6),    # DRATE_E=6 -> 2400 baud
    (REG_MDMCFG3, 0x83),    # DRATE_M -> 2399.5 baud
    (REG_MDMCFG2, 0x30),    # ASK/OOK, Manchester off (pre-encoded), no sync
    (REG_MDMCFG1, 0x00),
    (REG_MDMCFG0, 0xF8),
    (REG_DEVIATN, 0x00),    # unused for OOK
    (REG_MCSM1, 0x00),
    (REG_MCSM0, 0x04),      # manual calibration (we SCAL explicitly before TX)
    (REG_FREND1, 0x56),
    (REG_FREND0, 0x11),     # PA_POWER=1: PATABLE[0]='0' symbol, [1]='1' symbol
    (REG_FSCAL3, 0xEA),
    (REG_FSCAL2, 0x2A),
    (REG_FSCAL1, 0x00),
    (REG_FSCAL0, 0x11),
    (REG_TEST2, 0x81),
    (REG_TEST1, 0x35),
    (REG_TEST0, 0x09),
]

# PATABLE: index0 = '0' symbol (carrier OFF, MUST be 0x00), index1 = '1' symbol (carrier ON)
PA_TABLE = bytes([0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])


def compute_freq_regs(target_hz=TARGET_FREQ_HZ, xosc_hz=XOSC_HZ):
    freq_word = round(target_hz * (1 << 16) / xosc_hz)
    f2 = (freq_word >> 16) & 0xFF
    f1 = (freq_word >> 8) & 0xFF
    f0 = freq_word & 0xFF
    actual_hz = freq_word * xosc_hz / (1 << 16)
    return f2, f1, f0, actual_hz


class CC1101TX:
    def __init__(self, bus=0, device=0, spi_hz=500_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = spi_hz
        self.spi.mode = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(RAD_EN, GPIO.OUT)
        GPIO.setup(PWRGD, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # --- low level SPI ---
    def strobe(self, addr):
        return self.spi.xfer2([addr])[0]

    def write_reg(self, addr, value):
        self.spi.xfer2([addr & 0x3F, value])

    def read_reg(self, addr):
        return self.spi.xfer2([addr | 0x80, 0x00])[1]

    def read_status_reg(self, addr):
        return self.spi.xfer2([addr | 0xC0, 0x00])[1]

    def write_burst(self, addr, data: bytes):
        self.spi.xfer2([addr | 0x40] + list(data))

    def read_burst(self, addr, n):
        return bytes(self.spi.xfer2([addr | 0xC0] + [0] * n)[1:])

    # --- power sequencing ---
    def power_on(self, timeout_s=0.5, extra_settle_s=0.5):
        GPIO.output(RAD_EN, GPIO.HIGH)
        deadline = time.time() + timeout_s
        pwrgd_ok = False
        while time.time() < deadline:
            if GPIO.input(PWRGD):
                pwrgd_ok = True
                break
            time.sleep(0.005)
        if not pwrgd_ok:
            return False
        time.sleep(extra_settle_s)
        return True

    def power_off(self):
        GPIO.output(RAD_EN, GPIO.LOW)

    # --- register configuration ---
    def configure_ook_tx(self):
        self.strobe(SRES)
        time.sleep(0.1)

        for addr, value in CC1101_CONFIG:
            self.write_reg(addr, value)

        f2, f1, f0, actual_freq = compute_freq_regs()
        self.write_reg(REG_FREQ2, f2)
        self.write_reg(REG_FREQ1, f1)
        self.write_reg(REG_FREQ0, f0)

        self.strobe(SIDLE)
        self.strobe(SFTX)
        self.strobe(SFRX)

        self.write_burst(PATABLE, PA_TABLE)

        # verify PA table landed correctly
        pa_readback = self.read_burst(PATABLE, 2)
        if pa_readback[0] != 0x00 or pa_readback[1] != 0xC0:
            raise RuntimeError(f"PATABLE readback mismatch: {pa_readback.hex()} "
                                f"(expected 00c0) - '0' symbol not silent, or "
                                f"'1' symbol at wrong power")

        self.strobe(SCAL)

        return {"target_freq_hz": TARGET_FREQ_HZ, "actual_freq_hz": actual_freq}

    # --- transmit ---
    def transmit(self, payload: bytes, timeout_s=2.0):
        if len(payload) > 255:
            raise ValueError(f"Fixed-length mode caps PKTLEN at 255 bytes, got {len(payload)}")

        self.write_reg(REG_PKTLEN, len(payload))

        self.strobe(SIDLE)
        self.strobe(SFTX)

        self.strobe(SCAL)
        time.sleep(0.005)  # calibration takes ~720us; generous margin

        marc = self.get_marcstate()
        if marc != 0x01:  # IDLE
            self.strobe(SIDLE)
            time.sleep(0.001)
            marc = self.get_marcstate()
            if marc != 0x01:
                raise RuntimeError(f"Calibration failed to return to IDLE: MARCSTATE={marc:#04x}")

        first = payload[:FIFO_SIZE]
        self.write_burst(TXFIFO, first)
        pos = len(first)

        self.strobe(STX)

        deadline = time.time() + timeout_s
        while True:
            marc = self.get_marcstate()
            count, underflow = self.get_txbytes()

            if underflow or marc == 0x16:  # TXFIFO_UNDERFLOW state
                raise RuntimeError(f"TX error: MARCSTATE={marc:#04x} TXBYTES count={count} "
                                    f"underflow={underflow} at pos={pos}/{len(payload)}")

            if time.time() > deadline:
                raise RuntimeError(f"TX timeout: MARCSTATE={marc:#04x} pos={pos}/{len(payload)}")

            if pos < len(payload):
                free = FIFO_SIZE - count
                if free > 0:
                    chunk = payload[pos:pos + free]
                    self.write_burst(TXFIFO, chunk)
                    pos += len(chunk)
                continue

            # all bytes queued - done once FIFO drains and radio returns to IDLE
            if count == 0 and marc in (0x00, 0x01):
                break

        self.strobe(SIDLE)
        self.strobe(SFTX)

    def get_marcstate(self):
        return self.read_status_reg(MARCSTATE) & 0x1F

    def get_txbytes(self):
        val = self.read_status_reg(TXBYTES)
        return val & 0x7F, bool(val & 0x80)

    def close(self):
        self.spi.close()
        GPIO.cleanup()


if __name__ == "__main__":
    f2, f1, f0, actual_freq = compute_freq_regs()
    print(f"FREQ2={f2:#04x} FREQ1={f1:#04x} FREQ0={f0:#04x}  actual={actual_freq/1e6:.4f} MHz")
