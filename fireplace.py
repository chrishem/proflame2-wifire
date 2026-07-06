#!/usr/bin/env python3
"""
fireplace.py: Proflame 2 controller using CC1101 via SPI.

Reworked from johnellinwood/smartfire to replace rfcat/YardStick One
with the cc1101 Python library (pip install cc1101) for direct SPI
control of a CC1101 module on a Raspberry Pi.

Protocol notes (from smartfire README / FCC ID T99058402300):
  - Frequency  : 314.973 MHz
  - Modulation : OOK (ASK)
  - Baud rate  : 2400
  - Encoding   : Extended Thomas Manchester
  - Packet     : 3 serial words + 2 command words + 2 ECC words
  - Each word  : sync symbol + start guard + 9 data bits + parity + end guard
  - Transmission: 5 bursts per command

Pin assumptions (carrier board v2):
  - SPI CE0    : GPIO8  (pin 24)
  - RAD_EN     : GPIO16 (pin 36) -- MCP1727 ~SHDN~
  - PWRGD      : GPIO26 (pin 37) -- MCP1727 PWRGD
"""

import logging
import time
import RPi.GPIO as GPIO
import cc1101
from bitstring import Bits, BitArray

logger = logging.getLogger(__name__)

# ── Hardware config ───────────────────────────────────────────────────────────
PIN_RADIO_EN   = 16    # MCP1727 ~SHDN~ -- drive HIGH to power CC1101
PIN_POWER_GOOD = 26    # MCP1727 PWRGD  -- read to confirm 3v3radio is stable

FREQUENCY_HZ   = 314_973_000   # 314.973 MHz
BAUD_RATE      = 2400
TX_REPEAT      = 5             # protocol requires 5 transmissions per command

# ── Default serial (replace with your captured serial) ───────────────────────
DEFAULT_SERIAL = ['001001011', '011110100', '000000100']


class Fireplace:
    """Proflame 2 fireplace controller."""

    def __init__(self, serial=None):
        self._serial    = DEFAULT_SERIAL if serial is None else serial
        self._pilot     = True
        self._light     = 0
        self._thermostat = False
        self._power     = False
        self._front     = False
        self._fan       = 0
        self._aux       = False
        self._flame     = 0

        self._gpio_ready = False
        self._setup_gpio()

    # ── GPIO / power management ───────────────────────────────────────────────

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_RADIO_EN,   GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_POWER_GOOD, GPIO.IN)
        self._gpio_ready = True

    def _enable_radio(self, timeout=2.0):
        """Drive RAD_EN high and wait for PWRGD."""
        GPIO.output(PIN_RADIO_EN, GPIO.HIGH)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if GPIO.input(PIN_POWER_GOOD):
                logger.debug("CC1101 power good")
                time.sleep(0.05)   # brief settle
                return
            time.sleep(0.01)
        raise RuntimeError("CC1101 power good timeout — check MCP1727 and RAD_EN wiring")

    def _disable_radio(self):
        """Drive RAD_EN low to power down CC1101."""
        GPIO.output(PIN_RADIO_EN, GPIO.LOW)

    def cleanup(self):
        """Release GPIO resources."""
        self._disable_radio()
        GPIO.cleanup()

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def serial(self):
        return self._serial

    @property
    def pilot(self):
        return self._pilot

    @pilot.setter
    def pilot(self, value):
        self.set(pilot=value)

    @property
    def light(self):
        return self._light

    @light.setter
    def light(self, value):
        self.set(light=value)

    @property
    def thermostat(self):
        return self._thermostat

    @thermostat.setter
    def thermostat(self, value):
        self.set(thermostat=value)

    @property
    def power(self):
        return self._power

    @power.setter
    def power(self, value):
        self.set(power=value)

    @property
    def front(self):
        return self._front

    @front.setter
    def front(self, value):
        self.set(front=value)

    @property
    def fan(self):
        return self._fan

    @fan.setter
    def fan(self, value):
        self.set(fan=value)

    @property
    def aux(self):
        return self._aux

    @aux.setter
    def aux(self, value):
        self.set(aux=value)

    @property
    def flame(self):
        return self._flame

    @flame.setter
    def flame(self, value):
        self.set(flame=value)

    @property
    def state(self):
        return {
            'serial':     self.serial,
            'pilot':      self.pilot,
            'light':      self.light,
            'thermostat': self.thermostat,
            'power':      self.power,
            'front':      self.front,
            'fan':        self.fan,
            'aux':        self.aux,
            'flame':      self.flame,
        }

    # ── Command dispatch ──────────────────────────────────────────────────────

    def set(self, serial=None, pilot=None, light=None, thermostat=None,
            power=None, front=None, fan=None, aux=None, flame=None):
        """Update one or more fireplace parameters and transmit."""

        if pilot     is not None: self._pilot      = pilot
        if thermostat is not None: self._thermostat = thermostat
        if power     is not None: self._power      = power
        if front     is not None: self._front      = front
        if aux       is not None: self._aux        = aux

        if light is not None:
            if not (0 <= light <= 6):
                raise ValueError("light must be 0–6")
            self._light = light

        if fan is not None:
            if not (0 <= fan <= 6):
                raise ValueError("fan must be 0–6")
            self._fan = fan

        if flame is not None:
            if not (0 <= flame <= 6):
                raise ValueError("flame must be 0–6")
            self._flame = flame

        logger.info("State: power=%s flame=%s fan=%s light=%s pilot=%s thermostat=%s aux=%s front=%s",
                    self._power, self._flame, self._fan, self._light,
                    self._pilot, self._thermostat, self._aux, self._front)

        packet = self.build_packet()
        self.send_packet(packet)

    # ── Packet construction (unchanged from smartfire) ────────────────────────

    def build_packet(self):
        """Build the complete Manchester-encoded packet as a BitArray."""
        packet_words = []

        # 3-word serial number
        packet_words.extend([Bits(bin=s) for s in self.serial])

        # Command word 1: pilot | light(3) | 00 | thermostat | power | pad
        cmd1 = BitArray()
        cmd1.append('0b1' if self.pilot else '0b0')
        cmd1.append(Bits(uint=self.light, length=3))
        cmd1.append('0b00')
        cmd1.append('0b1' if self.thermostat else '0b0')
        cmd1.append('0b1' if self.power else '0b0')
        cmd1.append('0x0')   # padding
        packet_words.append(cmd1)

        # Command word 2: front | fan(3) | aux | flame(3) | pad
        cmd2 = BitArray()
        cmd2.append('0b1' if self.front else '0b0')
        cmd2.append(Bits(uint=self.fan, length=3))
        cmd2.append('0b1' if self.aux else '0b0')
        cmd2.append(Bits(uint=self.flame, length=3))
        cmd2.append('0x0')   # padding
        packet_words.append(cmd2)

        # ECC word 1
        ecc1 = BitArray()
        ecc1_high = (0xD ^ cmd1[0:4].uint ^ (cmd1[0:4].uint << 1) ^ (cmd1[4:8].uint << 1)) & 0xF
        ecc1_low  = cmd1[0:4].uint ^ cmd1[4:8].uint
        ecc1.append(Bits(uint=ecc1_high, length=4))
        ecc1.append(Bits(uint=ecc1_low,  length=4))
        ecc1.append('0x0')
        packet_words.append(ecc1)

        # ECC word 2
        ecc2 = BitArray()
        ecc2_high = (cmd2[0:4].uint ^ (cmd2[0:4].uint << 1) ^ (cmd2[4:8].uint << 1)) & 0xF
        ecc2_low  = cmd2[0:4].uint ^ cmd2[4:8].uint ^ 0x7
        ecc2.append(Bits(uint=ecc2_high, length=4))
        ecc2.append(Bits(uint=ecc2_low,  length=4))
        ecc2.append('0x0')
        packet_words.append(ecc2)

        # Build packet string with sync, guard, parity symbols
        packet_string = ''
        for word in packet_words:
            packet_string += 'S'            # sync symbol
            packet_string += '1'            # start guard
            packet_string += word[0:9].bin  # 9 data bits
            parity = word.count('0x1') % 2
            packet_string += Bits(uint=parity, length=1).bin
            packet_string += '1'            # end guard
        packet_string += 'Z' * 9           # burst separation padding

        # Extended Thomas Manchester encoding
        manchester = {'S': '11', '0': '01', '1': '10', 'Z': '00'}
        packet = BitArray()
        for b in packet_string:
            packet.append(Bits(bin=manchester[b]))

        logger.debug("Packet (%d bits): %s", len(packet), packet.hex)
        return packet

    # ── Transmission ──────────────────────────────────────────────────────────

    def send_packet(self, packet):
        """
        Transmit the packet TX_REPEAT times via CC1101.

        The cc1101 library handles OOK modulation and baud rate.
        packet.bytes is the raw Manchester-encoded bitstream.
        """
        self._enable_radio()

        try:
            with cc1101.CC1101() as radio:
                radio.set_base_frequency_hertz(FREQUENCY_HZ)
                radio.set_symbol_rate_baud(BAUD_RATE)
                radio.set_output_power(0)   # 0 dBm -- adjust if needed

                for i in range(TX_REPEAT):
                    radio.transmit(packet.bytes)
                    logger.debug("TX burst %d/%d", i + 1, TX_REPEAT)
                    time.sleep(0.05)        # inter-burst gap

            logger.info("Transmitted %d bursts", TX_REPEAT)

        finally:
            self._disable_radio()
