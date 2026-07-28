#!/usr/bin/env bash
#
# Quick smoke test for fpctrl.py using mosquitto_pub directly.
# Reads broker config from fpctrl.env (same file fpctrl.py uses).
#
# Run this while watching either:
#   journalctl -u fpctrl -f
# or:
#   mosquitto_sub -h <host> -t fireplace/state -v
# in another terminal to see what actually happens.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/fpctrl.env"

if [[ ! -f "$ENV_FILE" ]]; then
    echo "Can't find $ENV_FILE"
    exit 1
fi

# fpctrl.env is plain KEY=VALUE / # comments - safe to source directly.
source "$ENV_FILE"

MQTT_PORT="${MQTT_PORT:-1883}"
SET_TOPIC="fireplace/set"

AUTH_ARGS=()
if [[ -n "${MQTT_USERNAME:-}" ]]; then
    AUTH_ARGS+=(-u "$MQTT_USERNAME")
fi
if [[ -n "${MQTT_PASSWORD:-}" ]]; then
    AUTH_ARGS+=(-P "$MQTT_PASSWORD")
fi

send() {
    local payload="$1"
    local desc="$2"
    echo ""
    echo "--- $desc ---"
    echo "  -> $payload"
    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" "${AUTH_ARGS[@]}" -t "$SET_TOPIC" -q 1 -m "$payload"
    sleep 3
}

echo "=== fpctrl smoke test against $MQTT_HOST:$MQTT_PORT ==="

send '{"status": true}' "Status request - should publish current state, no hardware access"
send '{"power":"on","flame":3,"fan":2,"light":1}' "Full power-on command"
send '{"flame":5}' "Partial update - only flame changes, fan/light/power should carry forward"
send '{"backburner":"on"}' "Backburner on - flame/fan/light/power should carry forward"
send '{"flame":9}' "Invalid flame value (out of range) - should be rejected and logged, nothing sent to the radio"
send '{"pilot":"ipi"}' "Pilot change while power is (carried-forward) on - should be rejected"
send '{"power":"off"}' "Power off"
send '{"power":"off","pilot":"cpi"}' "Pilot change now that power is off - should succeed"
send '{"status": true}' "Final status check - confirm resting state"

echo ""
echo "=== Done. Check journalctl -u fpctrl and/or fireplace/state for results. ==="
