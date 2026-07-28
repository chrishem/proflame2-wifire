#!/usr/bin/env python3
"""
test_mqtt_status.py - quick sanity test for fpctrl.py's status request.

Subscribes to fireplace/state, publishes {"status": true} to fireplace/set,
and prints whatever comes back. Reads broker config from fpctrl.env (the
same file fpctrl.py itself uses) so there's nothing separate to configure.

Not part of fpctrl.py's dependency chain - this is a standalone throwaway
test, safe to run from a laptop/dev machine as long as it can reach the
broker, no hardware/root needed.

Usage:
  python3 test_mqtt_status.py
"""

import json
import os
import sys
import time

import paho.mqtt.client as mqtt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ENV_FILE = os.path.join(SCRIPT_DIR, "fpctrl.env")

STATE_TOPIC = "fireplace/state"
SET_TOPIC = "fireplace/set"

LISTEN_SECONDS = 10


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


def on_connect(client, userdata, flags, rc):
    if rc != 0:
        print(f"Connect failed, rc={rc}")
        sys.exit(1)
    print(f"Connected. Subscribing to '{STATE_TOPIC}'...")
    client.subscribe(STATE_TOPIC, qos=1)
    print(f"Publishing status request to '{SET_TOPIC}': {{'status': true}}")
    client.publish(SET_TOPIC, json.dumps({"status": True}), qos=1)


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        payload = msg.payload.decode(errors="replace")
    print(f"[{msg.topic}] {payload}")


def main():
    env = load_env()
    host = env.get("MQTT_HOST")
    if not host:
        print(f"MQTT_HOST not set in {ENV_FILE}")
        sys.exit(1)
    port = int(env.get("MQTT_PORT", "1883"))
    username = env.get("MQTT_USERNAME")
    password = env.get("MQTT_PASSWORD")

    client = mqtt.Client(client_id="fpctrl-status-test")
    if username:
        client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(host, port, keepalive=10)
    client.loop_start()

    print(f"Listening for {LISTEN_SECONDS}s (Ctrl+C to stop early)...")
    try:
        time.sleep(LISTEN_SECONDS)
    except KeyboardInterrupt:
        pass

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
