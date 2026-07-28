#!/usr/bin/env python3
"""
fpctrl - Proflame2 fireplace MQTT-driven daemon.

Long-running service, not a CLI tool. Subscribes to MQTT topic
'fireplace/set' for inbound commands and applies them via the CC1101,
persists the resulting state to fireplace_state.json, and publishes to
'fireplace/state' (retained) whenever the state actually changes. There is
intentionally no command-line control mode - if a CLI is wanted later, it
should be a separate thin tool that just publishes to 'fireplace/set'.

Command payload (any subset of these keys; omitted keys carry forward from
the last-known state, same principle as before - the real remote always
transmits its full state, so we have to too):

  {"power": "on"|"off"|true|false,
   "flame": 0-6,
   "fan": 0-6,
   "light": 0-6,
   "backburner": "on"|"off"|true|false,
   "pilot": "cpi"|"ipi"}

A payload containing a "status" key (any value) is a read-only request:
current state is published to 'fireplace/state' immediately, any other
fields in the same payload are ignored, and no radio/hardware access
happens. E.g. {"status": true}.

Validation rules (unchanged from the CLI version, now enforced against the
RESULTING merged state rather than just what's in a single payload):
  - flame must be 1-6 whenever the resulting power state is on (0 means "no
    target flame" - a correctly-executed no-op, blocked rather than sent).
  - pilot can only change when the resulting power state is off (the
    fireplace must be off to switch CPI/IPI - confirmed by the user, not
    just a software guess).

NeoPixel status is delegated entirely to npdaemon over its Unix domain
socket (/run/npdaemon/npdaemon.sock) - this process does not touch
rpi_ws281x/DMA/GPIO18 directly, and does NOT need root as a result. It does
need:
  - membership in the `spi` group (for /dev/spidev0.0)
  - membership in the `gpio` group (for RAD_EN/PWRGD via RPi.GPIO)
  - to run as the same user npdaemon's socket is chowned to (currently
    `tech`, per npdaemon's own design doc)
Check with `groups $USER`; add with `sudo usermod -aG spi,gpio <user>`
(requires a fresh login / service restart to take effect).

Failsafe: while the fireplace is on, an auto-shutoff timer is armed for
FIREPLACE_MAX_ON_MINUTES (fpctrl.env, default 60). ANY command received
while already on resets/extends this timer - same principle as a hot tub's
"jets running" session limit, but session-extending rather than a hard cap:
the fireplace can never be left on with no active supervising process, but
normal use (adjusting flame/fan) doesn't cut a session artificially short.
Set FIREPLACE_MAX_ON_MINUTES=0 to explicitly disable (not the default -
this is a safety feature, not something to silently opt out of).

MQTT_HOST in fpctrl.env is REQUIRED for this daemon (unlike the old
publish-only usage where it was optional) - it's the only control input.
Missing config is a fatal startup error, not a silent skip.

Pin map (per WiFirePi interposer board):
  RAD_EN (LDO enable, "LDOCTRL")  GPIO27
  PWRGD  (LDO power-good)          GPIO22
  CC1101 CHIPSEL                   GPIO8   (hardware SPI0 CE0)
  NeoPixel data                    GPIO18  (owned by npdaemon, not this process)
"""

import json
import logging
import os
import signal
import socket as socketlib
import sys
import threading
import time

import RPi.GPIO as GPIO

from proflame2_protocol import FireplaceState, ChecksumConstants, build_burst_bits, bits_to_bytes
from cc1101_tx import CC1101TX

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("fpctrl")

RAD_EN = 27
PWRGD = 22

SERIAL_NUMBER = 0xA3D502
CHECKSUM = ChecksumConstants(c1=0x7, d1=0x5, c2=0x4, d2=0xD)

LDO_SETTLE_S = 0.25

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(SCRIPT_DIR, "fireplace_state.json")
ENV_FILE = os.path.join(SCRIPT_DIR, "fpctrl.env")

MQTT_STATE_TOPIC = "fireplace/state"
MQTT_COMMAND_TOPIC = "fireplace/set"

NPDAEMON_SOCK = "/run/npdaemon/npdaemon.sock"

DEFAULT_STATE = {
    "power": False,
    "pilot_cpi": False,
    "flame": 0,
    "fan": 0,
    "light": 0,
    "front": False,
}

command_lock = threading.Lock()
_failsafe_timer = None
_failsafe_timer_lock = threading.Lock()

_shutdown_event = threading.Event()


# --- env / state file helpers ---

def load_env(path=ENV_FILE):
    env = {}
    if not os.path.exists(path):
        return env
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            env[key.strip()] = value.strip().strip('"').strip("'")
    return env


def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE) as f:
            return json.load(f)
    log.info(f"No state file at {STATE_FILE} - starting from baseline "
             f"(power off, everything 0/off). This is a guess, not a "
             f"confirmed reading of the real fireplace.")
    return dict(DEFAULT_STATE)


def save_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)


def canonical(state):
    """Reduce to just the fields we track, so stale keys (aux, thermostat
    from older versions) never trigger a false 'state changed' positive."""
    return {k: state.get(k) for k in DEFAULT_STATE}


# --- npdaemon NeoPixel client (fire-and-forget, never raises) ---

IDLE_COLOR = [0, 0, 60]
IDLE_BRIGHTNESS = 40


def np_send(payload: dict):
    try:
        with socketlib.socket(socketlib.AF_UNIX, socketlib.SOCK_STREAM) as s:
            s.settimeout(1.0)
            s.connect(NPDAEMON_SOCK)
            s.sendall(json.dumps(payload).encode() + b"\n")
    except Exception as e:
        log.warning(f"npdaemon unreachable ({e}) - NeoPixel status not updated")


def np_idle():
    np_send({"effect": "solid", "color": IDLE_COLOR, "brightness": IDLE_BRIGHTNESS})


def np_processing():
    np_send({"effect": "solid", "color": [255, 180, 0], "brightness": 120, "override": True})


def np_success():
    np_send({"effect": "pulse", "color": [0, 255, 0], "brightness": 120,
              "speed": 2.0, "duration": 1.5, "override": True})
    np_send({"effect": "solid", "color": IDLE_COLOR, "brightness": IDLE_BRIGHTNESS})  # queues behind pulse


def np_failsafe():
    np_send({"effect": "pulse", "color": [255, 140, 0], "brightness": 150,
              "speed": 2.0, "duration": 2.5, "override": True})
    np_send({"effect": "solid", "color": IDLE_COLOR, "brightness": IDLE_BRIGHTNESS})


def np_error():
    np_send({"effect": "blink", "color": [255, 0, 0], "brightness": 150,
              "interval": 0.2, "cycles": 6, "override": True})
    np_send({"effect": "solid", "color": IDLE_COLOR, "brightness": IDLE_BRIGHTNESS})


# --- MQTT state publish (retained, only on actual change) ---

def publish_mqtt_state(client, state):
    payload = json.dumps(state)
    # Deliberately NOT calling wait_for_publish() here: this function runs
    # inside on_message(), which executes on the same thread loop_forever()
    # uses to pump the network loop. Blocking here for a PUBACK would block
    # the very loop that processes that PUBACK - a self-deadlock that (before
    # this fix) silently ate the full 5s timeout on every single publish.
    # QoS 1 already gives delivery guarantees at the protocol level; we don't
    # need a synchronous confirmation on top of it.
    client.publish(MQTT_STATE_TOPIC, payload, qos=1, retain=True)
    log.info(f"Published state to {MQTT_STATE_TOPIC}: {payload}")


# --- command validation / merge (MQTT JSON -> merged state dict) ---

def _coerce_onoff(value, field_name):
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.lower() in ("on", "off"):
        return value.lower() == "on"
    raise ValueError(f"invalid {field_name} value: {value!r} (expected on/off or true/false)")


def _coerce_range(value, field_name, lo=0, hi=6):
    if isinstance(value, bool) or not isinstance(value, int) or not (lo <= value <= hi):
        raise ValueError(f"invalid {field_name} value: {value!r} (expected int {lo}-{hi})")
    return value


def validate_and_merge(payload: dict, persisted: dict):
    """Returns (merged_state, None) on success, or (None, error_message)."""
    merged = dict(persisted)
    merged.pop("aux", None)
    merged.pop("thermostat", None)

    try:
        if "power" in payload:
            merged["power"] = _coerce_onoff(payload["power"], "power")
        if "flame" in payload:
            merged["flame"] = _coerce_range(payload["flame"], "flame")
        if "fan" in payload:
            merged["fan"] = _coerce_range(payload["fan"], "fan")
        if "light" in payload:
            merged["light"] = _coerce_range(payload["light"], "light")
        if "backburner" in payload:
            merged["front"] = _coerce_onoff(payload["backburner"], "backburner")
        if "pilot" in payload:
            pilot_val = payload["pilot"]
            if not (isinstance(pilot_val, str) and pilot_val.lower() in ("cpi", "ipi")):
                raise ValueError(f"invalid pilot value: {pilot_val!r} (expected cpi/ipi)")
            if merged["power"]:
                raise ValueError("pilot mode can only be changed while power is off "
                                  "(resulting power state is on)")
            merged["pilot_cpi"] = (pilot_val.lower() == "cpi")
    except ValueError as e:
        return None, str(e)

    if merged["power"] and merged["flame"] == 0:
        return None, ("resulting flame=0 with power=on (0 means 'no target "
                       "flame' - a correctly-executed no-op, rejected instead "
                       "of silently doing nothing)")

    return merged, None


# --- failsafe timer ---

def get_max_on_minutes():
    env = load_env()
    raw = env.get("FIREPLACE_MAX_ON_MINUTES", "60")
    try:
        return int(raw)
    except ValueError:
        log.warning(f"FIREPLACE_MAX_ON_MINUTES={raw!r} is not an integer, defaulting to 60")
        return 60


def cancel_failsafe():
    global _failsafe_timer
    with _failsafe_timer_lock:
        if _failsafe_timer is not None:
            _failsafe_timer.cancel()
            _failsafe_timer = None


def arm_failsafe():
    global _failsafe_timer
    minutes = get_max_on_minutes()
    with _failsafe_timer_lock:
        if _failsafe_timer is not None:
            _failsafe_timer.cancel()
            _failsafe_timer = None
        if minutes <= 0:
            log.info("Failsafe disabled (FIREPLACE_MAX_ON_MINUTES=0)")
            return
        _failsafe_timer = threading.Timer(minutes * 60, failsafe_fire)
        _failsafe_timer.daemon = True
        _failsafe_timer.start()
        log.info(f"Failsafe (re)armed: auto-off in {minutes}m unless another command arrives first")


def failsafe_fire():
    log.warning("FAILSAFE: no command reset the timer within the configured window "
                "while the fireplace was on - auto-shutting off")
    with command_lock:
        persisted = load_state()
        merged = dict(persisted)
        merged["power"] = False
        apply_command(merged, mqtt_client, is_failsafe=True)


# --- core apply path (shared by MQTT commands and the failsafe) ---

def apply_command(merged, client, is_failsafe=False):
    """Assumes command_lock is already held by the caller."""
    persisted = load_state()
    state_changed = canonical(persisted) != canonical(merged)

    state = FireplaceState(
        power=merged["power"],
        pilot_cpi=merged["pilot_cpi"],
        flame=merged["flame"],
        fan=merged["fan"],
        light=merged["light"],
        front=merged["front"],
    )

    if not is_failsafe:
        np_processing()

    burst_bits = build_burst_bits(SERIAL_NUMBER, state, CHECKSUM)
    payload = bits_to_bytes(burst_bits)

    radio = None
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(RAD_EN, GPIO.OUT)
        GPIO.setup(PWRGD, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.output(RAD_EN, GPIO.HIGH)
        time.sleep(LDO_SETTLE_S)
        if not GPIO.input(PWRGD):
            raise RuntimeError("PWRGD is LOW - LDO did not come up")

        radio = CC1101TX()
        radio.configure_ook_tx()
        radio.transmit(payload)

        GPIO.output(RAD_EN, GPIO.LOW)

        save_state(merged)
        log.info(f"Applied command ({'FAILSAFE' if is_failsafe else 'MQTT'}): {merged}")

        if state_changed:
            publish_mqtt_state(client, merged)
        else:
            log.info("State unchanged - not re-publishing.")

        if is_failsafe:
            np_failsafe()
        else:
            np_success()

        if merged["power"]:
            arm_failsafe()
        else:
            cancel_failsafe()

    except Exception as e:
        log.error(f"Command failed: {e}")
        np_error()
        GPIO.output(RAD_EN, GPIO.LOW)
    finally:
        if radio is not None:
            radio.spi.close()
        GPIO.cleanup()


# --- MQTT wiring ---

mqtt_client = None


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log.info(f"Connected to MQTT broker, subscribing to {MQTT_COMMAND_TOPIC}")
        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)
        np_idle()
    else:
        log.error(f"MQTT connect failed, rc={rc}")


def on_disconnect(client, userdata, rc):
    log.warning(f"MQTT disconnected (rc={rc}) - paho will attempt to reconnect")


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except Exception as e:
        log.warning(f"Ignoring malformed payload on {msg.topic}: {e}")
        np_error()
        return
    if not isinstance(payload, dict):
        log.warning(f"Ignoring non-object payload on {msg.topic}: {payload!r}")
        np_error()
        return

    if "status" in payload:
        # Read-only request: report current state, ignore any other fields
        # in the same payload, never touches the radio/hardware.
        with command_lock:
            persisted = load_state()
            log.info(f"Status request received - publishing current state")
            publish_mqtt_state(client, persisted)
        return

    with command_lock:
        persisted = load_state()
        merged, err = validate_and_merge(payload, persisted)
        if err:
            log.warning(f"Rejected command {payload}: {err}")
            np_error()
            return
        apply_command(merged, client)


def main():
    global mqtt_client

    import paho.mqtt.client as mqtt

    env = load_env()
    host = env.get("MQTT_HOST")
    if not host:
        log.critical(f"MQTT_HOST is not set in {ENV_FILE} - this daemon has no "
                      f"control input without it. Exiting.")
        sys.exit(1)
    port = int(env.get("MQTT_PORT", "1883"))
    username = env.get("MQTT_USERNAME")
    password = env.get("MQTT_PASSWORD")
    client_id = env.get("MQTT_CLIENT_ID", "fpctrl")

    log.info(f"fpctrl starting - broker={host}:{port} "
             f"command_topic={MQTT_COMMAND_TOPIC} state_topic={MQTT_STATE_TOPIC} "
             f"failsafe={get_max_on_minutes()}m")

    client = mqtt.Client(client_id=client_id)
    if username:
        client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    mqtt_client = client

    def handle_signal(signum, frame):
        log.info(f"Received signal {signum}, shutting down...")
        _shutdown_event.set()
        cancel_failsafe()
        client.disconnect()
        GPIO.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    client.connect(host, port, keepalive=30)
    client.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
