"""
Proflame2 (SIT/Proflame 2) RF protocol encoder.

Protocol reference:
  - https://github.com/johnellinwood/smartfire (original reverse engineering)
  - https://github.com/j2deen/proflame2-esp (CC1101-native implementation + corrections)

This module is pure logic - no SPI/GPIO/hardware dependencies - so it can be
tested and verified independently of the radio.

Packet structure (per protocol docs, verified against a real capture from
this device - see test vector at the bottom of this file):

  7 words x 13 symbols = 91 symbols (decoded), each symbol Manchester-encoded
  to 2 raw chip-bits -> 182 raw bits per packet. Each command burst sends the
  same packet 5 times, separated by 12 zero-bits.

  Word format (13 symbols): S | 1 | 8 data bits | pad | parity | 1
    - S = sync symbol (raw '11')
    - guard bit = always 1
    - pad bit = 1 for word 1 (Serial 1) only, 0 for all other words
    - parity = 1 if (data bits + pad bit) has an odd number of 1s, else 0
    - end guard bit = always 1

  Word order: Serial1, Serial2, Serial3, Command1, Command2, Error1, Error2

  Serial 1/2/3 data bytes = the 24-bit serial number, split into 3 bytes,
  high byte first.

  Command1 data byte (MSB first): pilot(1) | light(3) | 0 | 0 | thermostat(1) | power(1)
  Command2 data byte (MSB first): front(1) | fan(3) | aux(1) | flame(3)

  Error1 = f(Command1, C1, D1); Error2 = f(Command2, C2, D2)
  where, with h/l = high/low nibble of the command byte:
    X = (C ^ ((h << 1) & 0xF) ^ h ^ ((l << 1) & 0xF)) & 0xF   # error high nibble
        (shift is truncated to 4 bits, not rotated)
        note: (l << 1) is intentional per the reference formula, even though
        the "low nibble shifted" term uses l, not the error word's own nibble
    Y = (D ^ h ^ l) & 0xF                                      # error low nibble
  C1/D1/C2/D2 are DEVICE SPECIFIC - must be derived from a real capture.
"""

from dataclasses import dataclass


# --- Manchester encoding table (decoded symbol -> 2 raw chip bits) ---
MANCHESTER = {
    "0": "01",
    "1": "10",
    "Z": "00",  # zero padding
    "S": "11",  # sync
}


def parity_bit(bits: str) -> str:
    """Odd parity: '1' if the number of 1s in `bits` is odd, else '0'."""
    return "1" if bits.count("1") % 2 == 1 else "0"


def build_word(data_byte: int, pad: int) -> str:
    """Build a single 13-symbol decoded word from an 8-bit data value and pad bit."""
    data_bits = format(data_byte & 0xFF, "08b")
    pad_bit = "1" if pad else "0"
    par = parity_bit(data_bits + pad_bit)
    return "S" + "1" + data_bits + pad_bit + par + "1"


def compute_error_byte(command_byte: int, c: int, d: int) -> int:
    """Compute the error-detection byte for a command byte using device-specific C/D."""
    h = (command_byte >> 4) & 0xF
    l = command_byte & 0xF
    x = (c ^ ((h << 1) & 0xF) ^ h ^ ((l << 1) & 0xF)) & 0xF
    y = (d ^ h ^ l) & 0xF
    return (x << 4) | y


@dataclass
class ChecksumConstants:
    c1: int
    d1: int
    c2: int
    d2: int


@dataclass
class FireplaceState:
    power: bool = False
    pilot_cpi: bool = False       # True = CPI, False = IPI
    thermostat: bool = False
    light: int = 0                # 0-6
    front: bool = False           # front flame / flame split
    fan: int = 0                  # 0-6
    aux: bool = False
    flame: int = 0                # 0-6

    def command1_byte(self) -> int:
        assert 0 <= self.light <= 6, "light must be 0-6 (7 is not allowed)"
        return (
            (1 if self.pilot_cpi else 0) << 7
            | (self.light & 0x7) << 4
            | 0 << 3
            | 0 << 2
            | (1 if self.thermostat else 0) << 1
            | (1 if self.power else 0)
        )

    def command2_byte(self) -> int:
        assert 0 <= self.fan <= 6, "fan must be 0-6"
        assert 0 <= self.flame <= 6, "flame must be 0-6"
        return (
            (1 if self.front else 0) << 7
            | (self.fan & 0x7) << 4
            | (1 if self.aux else 0) << 3
            | (self.flame & 0x7)
        )


def build_packet_symbols(serial_number: int, state: FireplaceState, checksum: ChecksumConstants) -> str:
    """Build the full 91-symbol decoded packet for a given state."""
    serial_bytes = [
        (serial_number >> 16) & 0xFF,
        (serial_number >> 8) & 0xFF,
        serial_number & 0xFF,
    ]

    cmd1 = state.command1_byte()
    cmd2 = state.command2_byte()
    err1 = compute_error_byte(cmd1, checksum.c1, checksum.d1)
    err2 = compute_error_byte(cmd2, checksum.c2, checksum.d2)

    words = [
        build_word(serial_bytes[0], pad=1),   # Serial 1 (pad=1, only word with pad set)
        build_word(serial_bytes[1], pad=0),   # Serial 2
        build_word(serial_bytes[2], pad=0),   # Serial 3
        build_word(cmd1, pad=0),              # Command 1
        build_word(cmd2, pad=0),              # Command 2
        build_word(err1, pad=0),              # Error 1
        build_word(err2, pad=0),              # Error 2
    ]
    return "".join(words)


def symbols_to_raw_bits(symbols: str) -> str:
    """Manchester-encode a decoded symbol string into raw on-air chip bits."""
    return "".join(MANCHESTER[s] for s in symbols)


def build_burst_bits(serial_number: int, state: FireplaceState, checksum: ChecksumConstants,
                      repeats: int = 5, separator_zero_bits: int = 12) -> str:
    """Build the full raw bitstream for one command burst (packet repeated N times)."""
    packet_symbols = build_packet_symbols(serial_number, state, checksum)
    packet_bits = symbols_to_raw_bits(packet_symbols)
    separator = "0" * separator_zero_bits
    parts = [packet_bits] * repeats
    return separator.join(parts)


def bits_to_bytes(bits: str) -> bytes:
    """Pack a raw bit string into bytes, MSB first, padding the final byte with 0s."""
    pad_len = (-len(bits)) % 8
    bits_padded = bits + "0" * pad_len
    return bytes(
        int(bits_padded[i:i + 8], 2) for i in range(0, len(bits_padded), 8)
    )


if __name__ == "__main__":
    # --- Regression test: reproduce the verified "Ohi" capture from this device ---
    # Captured & parity-validated 2026-07-21 (4/4 repeats decoded identically,
    # all 7 words pass parity independently).
    SERIAL = 0xA3D502
    CHECKSUM = ChecksumConstants(c1=0x7, d1=0x5, c2=0x4, d2=0xD)

    # "Ohi" = Output Hi: power on, flame=6 (high), fan=4, front on, thermostat off, light=0
    ohi_state = FireplaceState(
        power=True, pilot_cpi=True, thermostat=False, light=0,
        front=True, fan=4, aux=False, flame=6,
    )

    expected = (
        "S110100011111S111010101011S100000010011"
        "S110000001001S111000110001S111011100011S111000111011"
    )
    actual = build_packet_symbols(SERIAL, ohi_state, CHECKSUM)

    print("Expected:", expected)
    print("Actual:  ", actual)
    print("MATCH:", actual == expected)
    assert actual == expected, "Encoder does not reproduce the verified capture!"

    # --- Second test vector: the real remote's actual Power On command ---
    # Decoded from On_r.sub (2026-07-24), 4/4 repeats agree (1.000 score after
    # correcting the initial garbled read - see conversation history). Critically,
    # the real remote sends pilot_cpi=True on power-on, not False.
    real_power_on_expected_cmd1 = "10000001"  # pilot=1(CPI), light=0, thermostat=0, power=1
    real_power_on_state = FireplaceState(power=True, pilot_cpi=True)
    real_cmd1 = format(real_power_on_state.command1_byte(), "08b")
    print()
    print("Real remote Command1 (Power On):", real_power_on_expected_cmd1)
    print("Our encoder Command1 (power=True, pilot_cpi=True):", real_cmd1)
    assert real_cmd1 == real_power_on_expected_cmd1, (
        "pilot_cpi=True should reproduce the real remote's Power On Command1 byte!"
    )

    # --- Now build the actual command requested: Power On ---
    # Minimal state: just power on, everything else at a safe default (off/0).
    power_on_state = FireplaceState(power=True, pilot_cpi=True)
    burst_bits = build_burst_bits(SERIAL, power_on_state, CHECKSUM)
    burst_bytes = bits_to_bytes(burst_bits)

    print()
    print("Power-on packet symbols:", build_packet_symbols(SERIAL, power_on_state, CHECKSUM))
    print(f"Power-on burst: {len(burst_bits)} raw bits -> {len(burst_bytes)} bytes")
    print("Burst hex:", burst_bytes.hex())

