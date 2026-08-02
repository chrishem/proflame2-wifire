# WiFirePi Fireplace Bridge — Design & Current State

Raspberry Pi + CC1101 bridge that controls a SIT Proflame 2 gas fireplace
insert over 315MHz OOK, driven by MQTT so it can integrate with Matterbridge/
Alexa/voice control. Runs as an unprivileged daemon (`fpctrl.py`) alongside a
separate shared NeoPixel status daemon (`npdaemon`).

---

## Architecture

```
                    MQTT broker (ns2020.dim-x.net)
                    /                          \
        fireplace/set                    fireplace/state
        (subscribe)                      (publish, retained)
                    \                          /
                     fpctrl.py  (User=tech, no root)
                     /        |         \
           cc1101_tx.py  proflame2_    Unix domain socket
           (SPI/GPIO)    protocol.py   /run/npdaemon/npdaemon.sock
                         (pure logic)         |
                                          npdaemon (User=root)
                                               |
                                          NeoPixel (GPIO18)
```

- **`fpctrl.py`** is the only process that talks to the CC1101 radio. It has
  no command-line control mode — MQTT is its sole input.
- **`npdaemon`** is a separate, independently-designed project (its own repo)
  that owns the NeoPixel exclusively, so `fpctrl.py` doesn't need root/DMA
  access. `fpctrl.py` is just one of potentially several local clients that
  can set pixel state over its Unix socket.
- **`proflame2_protocol.py`** and **`cc1101_tx.py`** deliberately don't know
  about each other — one builds packets, the other transmits arbitrary
  bytes. Neither knows MQTT exists. `fpctrl.py` is the only file that wires
  all three together.

---

## Files (tracked in git)

| File | Role |
|---|---|
| `fpctrl.py` | The daemon. Subscribes to `fireplace/set`, validates/merges commands against persisted state, transmits via CC1101, publishes to `fireplace/state` on change, drives the failsafe timer, reports status to npdaemon. |
| `proflame2_protocol.py` | Pure protocol logic - no hardware/network dependencies. `FireplaceState` dataclass, packet/checksum/Manchester encoding, burst assembly. Self-tests on `python3 proflame2_protocol.py` against two independently-verified real captures. |
| `cc1101_tx.py` | CC1101 SPI/GPIO driver, TX only. Register table and TX sequence ported directly from `proflame2-esp`'s verified C++ source (fixed-length packet mode, manual calibration, tight-poll FIFO refill). |
| `test_power_on.py` | Frozen, hardware-only debugging tool (one-shot, no MQTT/broker needed, drives the NeoPixel directly via `rpi_ws281x`). Kept deliberately - useful for testing radio/driver changes in isolation from the MQTT daemon. |
| `test_mqtt_status.py` | Standalone MQTT sanity check - subscribes to `fireplace/state`, publishes `{"status": true}` to `fireplace/set`, prints the response. No hardware/root needed; runnable from any machine that can reach the broker. |
| `test_fp.sh` | `mosquitto_pub`-based smoke test script exercising the command set end-to-end (status, full command, partial update, invalid value rejection, pilot/power constraint, dedupe-on-no-change). |
| `cc1101test.py` | Early bring-up/diagnostic script (SPI register sanity check, LDO on/off/PWRGD sequence). Standalone hardware check, unrelated to the MQTT daemon. |
| `fpctrl.env.example` | Template for `fpctrl.env` (gitignored - contains broker credentials). |
| `fpctrl.service` | systemd unit, runs as `User=tech` (not root). |
| `requirements.txt` | Pruned to the four actual runtime dependencies (see below). |
| `.gitignore` | Excludes `.venv/`, `__pycache__/`, `fireplace_state.json` (runtime data), `fpctrl.env` (credentials), `smartfire/` (reference clone with its own nested `.git`). |

**Removed during cleanup:** `fireplace.py`, `server.py` (early dead Flask/
smartfire-based attempt, superseded), `run_fireplace_tests.sh` (targeted a
CLI mode that no longer exists). `state.py` was actually a duplicate of
`test_mqtt_status.py` under the wrong filename - renamed, not deleted.

**Not tracked, exists locally only:** `fireplace_state.json` (persisted
state), `fpctrl.env` (real broker credentials), `smartfire/` (reference
clone of johnellinwood/smartfire, kept for reference, not part of the
build).

---

## Dependencies

```
paho-mqtt==2.1.0
RPi.GPIO==0.7.1
rpi_ws281x==5.0.0
spidev==3.8
```

`rpi_ws281x` is only needed for `test_power_on.py` - `fpctrl.py` itself
doesn't touch it (NeoPixel delegated to npdaemon). Pruned during cleanup:
the Flask stack (was only for the now-removed `server.py`), the `cc1101`
PyPI package (unused - our own driver was built from scratch against
`proflame2-esp`'s verified register table, not this library), and its
`bitstring`/`bitarray`/`tibs` chain.

---

## Protocol (Proflame2 / SIT)

314.973 MHz, OOK/ASK, 2400 baud, Manchester variant. 7 words × 13 bits per
packet, 5× repeat burst. Full bit-level structure independently confirmed
against three separate implementations: `johnellinwood/smartfire` (original
spec), `j2deen/proflame2-esp` (source of the verified CC1101 register
table), and `rtl_433`'s built-in decoder (`src/devices/proflame2.c`).

- **This device's serial number:** `0xA3D502`
- **This device's checksum constants:** `C1=0x7, D1=0x5, C2=0x4, D2=0xD`
  (cross-validated against two real captures with different command values)

### Field semantics (unit-specific findings)

- `power` - **absolute state, not a toggle** (confirmed via real-world
  `rtl_433` community captures showing it held steady across many other
  field changes in the same session).
- `flame=0` while `power=1` is treated as invalid and rejected - no
  real-world capture from any source ever shows that combination; it likely
  means "no target flame," a silent no-op rather than an error.
- `pilot` (CPI/IPI) - a deliberate, separate seasonal setting. **Can only be
  changed while the resulting power state is off** (user-confirmed hardware
  constraint, enforced in `fpctrl.py`'s validation).
- **`backburner`** (user-facing name) - the protocol calls this bit `front`;
  confirmed via direct hardware testing that on this specific unit it
  drives the rear/back-row burner, not anything front-facing. Library code
  uses the internal implementation name "backburner" instead of the protocol-
  true "front".
- `aux` (Command2 bit 3) - tested extensively, confirmed no observable
  effect on this unit. Removed from the CLI/MQTT schema entirely; the
  dataclass field still exists (always `False`) for protocol completeness.
- `thermostat` - not used on the real remote for this fireplace; not
  exposed in `FireplaceState` usage here, though the protocol supports it.
- `light`/`fan` - at least one other product line's official manual lists
  these as "(not used)" despite being protocol-supported fields. Not
  confirmed either way for this specific unit.

---

## MQTT interface

| Topic | Direction | Payload |
|---|---|---|
| `fireplace/set` | Subscribe (commands) | JSON, any subset of `power`, `flame`, `fan`, `light`, `backburner`, `pilot`. Omitted fields carry forward from last-known state. A `"status"` key makes it a read-only request (no hardware access, just re-publishes current state). |
| `fireplace/state` | Publish (retained, QoS 1) | Full merged state as JSON. Only published when the state actually changes. |

Broker connection details (`MQTT_HOST`/`PORT`/`USERNAME`/`PASSWORD`/
`CLIENT_ID`) and `FIREPLACE_MAX_ON_MINUTES` come from `fpctrl.env`.

## Failsafe

While the fireplace is on, an auto-shutoff timer is armed for
`FIREPLACE_MAX_ON_MINUTES` (default 60). **Any** command received while
already on resets/extends the timer - hot-tub-style session-limit
semantics, not a hard cap from power-on. If nothing arrives within the
window, `fpctrl.py` sends its own power-off command through the same code
path as a normal MQTT command, logs it distinctly (`FAILSAFE:` prefix), and
signals a distinct amber pulse on the NeoPixel. Set to `0` to disable
(explicit opt-out only, not the default).

---

## Deployment

```bash
sudo usermod -aG spi,gpio tech   # if not already members (check: groups tech)
sudo ln -s /home/tech/projects/wifire/fpctrl.service /etc/systemd/system/fpctrl.service
sudo systemctl daemon-reload
sudo systemctl enable --now fpctrl.service
journalctl -u fpctrl -f
```

Requires `npdaemon.service` to be running for NeoPixel status (not a hard
dependency - `fpctrl.py` degrades gracefully and just logs a warning if the
socket is unreachable, per `np_send()`).

---

## Known open items

- `light`/`fan` real-world effect on this unit still unconfirmed.
- No RTL-SDR currently available for live `rtl_433` verification (worked
  around via Flipper Zero captures + custom decode tooling throughout this
  project; would still be useful to have for future debugging).
- No CLI control tool currently exists for `fpctrl.py` beyond raw
  `mosquitto_pub`/`test_fp.sh`. If wanted, should be a thin separate tool
  that just publishes to `fireplace/set` - not part of `fpctrl.py` itself.
- RX subsystem (listening for the physical remote's own transmissions to
  keep state in sync with manually-initiated changes) is design-only, not
  built. Would need the CC1101's hardware sync-word feature (loaded with
  this device's known serial number) rather than software bit-banging, to
  avoid the timing-jitter problems already solved on the TX side.
- `paho-mqtt` v1-style callback API is in use against a v2 library
  (`paho-mqtt==2.1.0`) - triggers a `DeprecationWarning` at startup, not
  currently broken but worth migrating to `CallbackAPIVersion.VERSION2`
  before a future `paho-mqtt` release drops v1 support entirely.
