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
  Note: "front" is a generic term above; in my specific case, the burner
        referred to by "front" is actually the BACK burner.

  Error1 = f(Command1, C1, D1); Error2 = f(Command2, C2, D2)
  where, with h/l = high/low nibble of the command byte:
    X = (C ^ ((h << 1) & 0xF) ^ h ^ ((l << 1) & 0xF)) & 0xF   # error high nibble
        (shift is truncated to 4 bits, not rotated)
        note: (l << 1) is intentional per the reference formula, even though
        the "low nibble shifted" term uses l, not the error word's own nibble
    Y = (D ^ h ^ l) & 0xF                                      # error low nibble
  C1/D1/C2/D2 are DEVICE SPECIFIC - must be derived from a real capture.
"""

from dataclasses import dataclass, field
from typing import Optional, List


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
    backburner: bool = False       # CONFIRMED via direct testing (2026-07-24): on THIS
                                   # unit, this bit controls the back-row/secondary
                                   # burner - despite the protocol calling it "front".
                                   # This is unit-specific wiring, not a universal
                                   # Proflame2 meaning - other installations may differ.
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
            (1 if self.backburner else 0) << 7
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


def bytes_to_bits(data: bytes) -> str:
    """Inverse of bits_to_bytes - unpack bytes into a raw bit string, MSB first."""
    return "".join(format(b, "08b") for b in data)


# --- RX / decode side (added for the RX subsystem - not yet hardware-tested) ---

RAW_TO_SYMBOL = {v: k for k, v in MANCHESTER.items()}  # "01"->"0", "10"->"1", "00"->"Z", "11"->"S"


def raw_bits_to_symbols(raw_bits: str) -> str:
    """Inverse Manchester decode: 2 raw chip-bits -> 1 decoded symbol.

    Raises ValueError on a bit pair with no Manchester meaning - this
    shouldn't happen on a clean capture, and is a useful integrity signal
    (a real bit-slip / noise glitch) distinct from a parity/checksum failure.
    """
    if len(raw_bits) % 2 != 0:
        raise ValueError(f"raw bit count ({len(raw_bits)}) is not even - can't be Manchester pairs")
    symbols = []
    for i in range(0, len(raw_bits), 2):
        pair = raw_bits[i:i + 2]
        if pair not in RAW_TO_SYMBOL:
            raise ValueError(f"invalid Manchester pair {pair!r} at raw bit offset {i}")
        symbols.append(RAW_TO_SYMBOL[pair])
    return "".join(symbols)


@dataclass
class DecodedWord:
    data_byte: int
    pad_bit: int
    parity_ok: bool
    guard_ok: bool     # both guard bits (position 1 and position 12) were '1'
    sync_ok: bool       # position 0 was the 'S' symbol


def decode_word(symbols_13: str) -> DecodedWord:
    """Inverse of build_word(). Expects exactly 13 decoded symbols."""
    if len(symbols_13) != 13:
        raise ValueError(f"expected 13 symbols, got {len(symbols_13)}")

    sync_ok = symbols_13[0] == "S"
    guard1 = symbols_13[1]
    data_bits = symbols_13[2:10]
    pad_bit_sym = symbols_13[10]
    parity_sym = symbols_13[11]
    end_guard = symbols_13[12]

    guard_ok = (guard1 == "1") and (end_guard == "1")

    if any(s not in ("0", "1") for s in data_bits + pad_bit_sym):
        raise ValueError(f"non-binary symbol in data/pad field: {symbols_13!r}")

    data_byte = int(data_bits, 2)
    pad_bit = int(pad_bit_sym)
    expected_parity = parity_bit(data_bits + pad_bit_sym)
    parity_ok = (parity_sym == expected_parity)

    return DecodedWord(data_byte=data_byte, pad_bit=pad_bit,
                        parity_ok=parity_ok, guard_ok=guard_ok, sync_ok=sync_ok)


def verify_error_byte(command_byte: int, error_byte: int, c: int, d: int) -> bool:
    """Inverse check of compute_error_byte() - True if error_byte matches
    what we'd compute for command_byte under these device-specific C/D."""
    return compute_error_byte(command_byte, c, d) == error_byte


@dataclass
class DecodedBurst:
    valid: bool                   # overall pass/fail - True only if every check below passed
    serial_number: Optional[int] = None
    command1: Optional[int] = None
    command2: Optional[int] = None
    checksum1_ok: bool = False
    checksum2_ok: bool = False
    words_ok: bool = False        # all 7 words passed sync/guard/parity
    errors: List[str] = field(default_factory=list)  # human-readable list of what failed, if any


def decode_packet_symbols(symbols_91: str, checksum: ChecksumConstants) -> DecodedBurst:
    """Decode one 91-symbol packet (7 words) back into serial number + command
    bytes, verifying guard/parity per word and the checksum per command byte.

    This is the RX-side integrity gate referenced in the RX subsystem design:
    a decode is only trusted (`.valid == True`) if every structural check
    passes, not just checksum - a bit-slip that happens to still produce a
    plausible checksum should still be caught by a guard/parity mismatch.
    """
    errors = []
    if len(symbols_91) != 91:
        return DecodedBurst(valid=False, errors=[f"expected 91 symbols, got {len(symbols_91)}"])

    word_strs = [symbols_91[i:i + 13] for i in range(0, 91, 13)]
    words = []
    for i, w in enumerate(word_strs):
        try:
            dw = decode_word(w)
        except ValueError as e:
            errors.append(f"word {i}: {e}")
            return DecodedBurst(valid=False, errors=errors)
        words.append(dw)
        if not dw.sync_ok:
            errors.append(f"word {i}: sync symbol missing/corrupt")
        if not dw.guard_ok:
            errors.append(f"word {i}: guard bit(s) wrong")
        if not dw.parity_ok:
            errors.append(f"word {i}: parity mismatch")

    words_ok = not errors

    serial1, serial2, serial3, cmd1, cmd2, err1, err2 = (w.data_byte for w in words)
    serial_number = (serial1 << 16) | (serial2 << 8) | serial3

    checksum1_ok = verify_error_byte(cmd1, err1, checksum.c1, checksum.d1)
    checksum2_ok = verify_error_byte(cmd2, err2, checksum.c2, checksum.d2)
    if not checksum1_ok:
        errors.append(f"Command1 checksum mismatch (got err1=0x{err1:02X})")
    if not checksum2_ok:
        errors.append(f"Command2 checksum mismatch (got err2=0x{err2:02X})")

    valid = words_ok and checksum1_ok and checksum2_ok

    return DecodedBurst(
        valid=valid,
        serial_number=serial_number,
        command1=cmd1,
        command2=cmd2,
        checksum1_ok=checksum1_ok,
        checksum2_ok=checksum2_ok,
        words_ok=words_ok,
        errors=errors,
    )


def command_bytes_to_state(cmd1: int, cmd2: int) -> FireplaceState:
    """Inverse of FireplaceState.command1_byte()/command2_byte()."""
    return FireplaceState(
        pilot_cpi=bool((cmd1 >> 7) & 1),
        light=(cmd1 >> 4) & 0x7,
        thermostat=bool((cmd1 >> 1) & 1),
        power=bool(cmd1 & 1),
        backburner=bool((cmd2 >> 7) & 1),
        fan=(cmd2 >> 4) & 0x7,
        aux=bool((cmd2 >> 3) & 1),
        flame=cmd2 & 0x7,
    )


if __name__ == "__main__":
    # --- Regression test: reproduce the verified "Ohi" capture from this device ---
    # Captured & parity-validated 2026-07-21 (4/4 repeats decoded identically,
    # all 7 words pass parity independently).
    SERIAL = 0xA3D502
    CHECKSUM = ChecksumConstants(c1=0x7, d1=0x5, c2=0x4, d2=0xD)

    # "Ohi" = Output Hi: power on, flame=6 (high), fan=4, backburner on, thermostat off, light=0
    ohi_state = FireplaceState(
        power=True, pilot_cpi=True, thermostat=False, light=0,
        backburner=True, fan=4, aux=False, flame=6,
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

    # --- RX decode round-trip test - proves the new decode logic added for
    # the RX subsystem is a true inverse of the encoder, using this device's
    # real serial/checksum constants, before it's ever run against hardware. ---
    print()
    print("--- RX decode round-trip test ---")
    roundtrip_state = FireplaceState(power=True, pilot_cpi=True, light=3,
                                      backburner=True, fan=4, flame=6)
    rt_symbols = build_packet_symbols(SERIAL, roundtrip_state, CHECKSUM)
    rt_decoded = decode_packet_symbols(rt_symbols, CHECKSUM)
    rt_recovered = command_bytes_to_state(rt_decoded.command1, rt_decoded.command2)
    print("Decoded valid:", rt_decoded.valid)
    print("Original: ", roundtrip_state)
    print("Recovered:", rt_recovered)
    assert rt_decoded.valid, f"Round-trip decode should be valid, errors={rt_decoded.errors}"
    assert rt_decoded.serial_number == SERIAL, "Round-trip serial mismatch!"
    assert rt_recovered == roundtrip_state, "Round-trip state mismatch!"
    print("ROUND-TRIP MATCH: True")
