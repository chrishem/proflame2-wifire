#!/usr/bin/env python3
"""
server.py: Flask REST server for the Proflame 2 fireplace controller.
"""

import logging
import RPi.GPIO as GPIO
from flask import Flask, request, jsonify, abort
from fireplace import Fireplace

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
fp  = Fireplace()

# ── GPIO state ────────────────────────────────────────────────────────────────

_gpio_pins = {}


def _pin_state(pin):
    """Get or initialize state for a GPIO pin. Default: floating input."""
    if pin not in _gpio_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        _gpio_pins[pin] = {
            'mode':     'input',
            'pull':     'off',
            'state':    False,
            'pwm':      None,
            'pwm_freq': 0,
            'pwm_duty': 0,
        }
    return _gpio_pins[pin]


def _cleanup_pwm(ps):
    """Stop PWM on a pin if running."""
    if ps['pwm'] is not None:
        ps['pwm'].stop()
        ps['pwm']      = None
        ps['pwm_freq'] = 0
        ps['pwm_duty'] = 0


# ── Helpers ───────────────────────────────────────────────────────────────────

def parse_bool(data):
    s = data.decode().strip().lower()
    if s in ('true', '1'):
        return True
    if s in ('false', '0'):
        return False
    abort(400, f"Expected true/false, got: {s}")


def parse_int(data, lo=0, hi=6):
    try:
        v = int(data.decode().strip())
    except ValueError:
        abort(400, f"Expected integer {lo}–{hi}")
    if not (lo <= v <= hi):
        abort(400, f"Value {v} out of range {lo}–{hi}")
    return v


# ── API index ─────────────────────────────────────────────────────────────────

@app.route("/", methods=['GET'])
def index():
    return jsonify({
        "wifire": "Proflame 2 Fireplace Controller",
        "endpoints": {
            "fireplace": {
                "GET  /state":       "Full fireplace state",
                "PUT  /state":       "Set multiple params (JSON body)",
                "GET  /serial":      "Remote serial number",
                "GET  /power":       "Power state",
                "PUT  /power":       "Set power (true/false)",
                "GET  /flame":       "Flame level (0-6)",
                "PUT  /flame":       "Set flame level (0-6)",
                "GET  /fan":         "Fan level (0-6)",
                "PUT  /fan":         "Set fan level (0-6)",
                "GET  /light":       "Light level (0-6)",
                "PUT  /light":       "Set light level (0-6)",
                "GET  /pilot":       "Pilot state",
                "PUT  /pilot":       "Set pilot (true/false)",
                "GET  /thermostat":  "Thermostat state",
                "PUT  /thermostat":  "Set thermostat (true/false)",
                "GET  /aux":         "Aux power state",
                "PUT  /aux":         "Set aux power (true/false)",
                "GET  /front":       "Front flame state",
                "PUT  /front":       "Set front flame (true/false)",
            },
            "gpio": {
                "GET  /gpio/<pin>/mode":     "Get pin mode (input/output/pwm)",
                "PUT  /gpio/<pin>/mode":     "Set pin mode (body: input/output/pwm)",
                "GET  /gpio/<pin>/pull":     "Get pull resistor (up/down/off)",
                "PUT  /gpio/<pin>/pull":     "Set pull resistor, input mode only (body: up/down/off)",
                "GET  /gpio/<pin>/state":    "Get output state, output mode only",
                "PUT  /gpio/<pin>/state":    "Set output high/low, output mode only (body: true/false)",
                "GET  /gpio/<pin>/reading":  "Read actual pin level (any mode)",
                "GET  /gpio/<pin>/pwm":      "Get PWM settings, pwm mode only",
                "PUT  /gpio/<pin>/pwm":      "Set PWM, pwm mode only (body: {frequency, duty_cycle})",
                "DELETE /gpio/<pin>/pwm":    "Stop PWM",
            }
        }
    })


# ── Fireplace routes ──────────────────────────────────────────────────────────

@app.route("/state", methods=['GET', 'PUT'])
def state():
    if request.method == 'GET':
        return jsonify(fp.state)
    data = request.get_json(force=True)
    fp.set(**{k: v for k, v in data.items() if k in
              ('pilot', 'light', 'thermostat', 'power', 'front', 'fan', 'aux', 'flame')})
    return jsonify(fp.state)


@app.route("/serial", methods=['GET'])
def serial():
    return jsonify({'serial': fp.serial})


@app.route("/power", methods=['GET', 'PUT'])
def power():
    if request.method == 'GET':
        return jsonify({'power': fp.power})
    fp.power = parse_bool(request.data)
    return jsonify({'power': fp.power})


@app.route("/flame", methods=['GET', 'PUT'])
def flame():
    if request.method == 'GET':
        return jsonify({'flame': fp.flame})
    fp.flame = parse_int(request.data)
    return jsonify({'flame': fp.flame})


@app.route("/fan", methods=['GET', 'PUT'])
def fan():
    if request.method == 'GET':
        return jsonify({'fan': fp.fan})
    fp.fan = parse_int(request.data)
    return jsonify({'fan': fp.fan})


@app.route("/light", methods=['GET', 'PUT'])
def light():
    if request.method == 'GET':
        return jsonify({'light': fp.light})
    fp.light = parse_int(request.data)
    return jsonify({'light': fp.light})


@app.route("/pilot", methods=['GET', 'PUT'])
def pilot():
    if request.method == 'GET':
        return jsonify({'pilot': fp.pilot})
    fp.pilot = parse_bool(request.data)
    return jsonify({'pilot': fp.pilot})


@app.route("/thermostat", methods=['GET', 'PUT'])
def thermostat():
    if request.method == 'GET':
        return jsonify({'thermostat': fp.thermostat})
    fp.thermostat = parse_bool(request.data)
    return jsonify({'thermostat': fp.thermostat})


@app.route("/aux", methods=['GET', 'PUT'])
def aux():
    if request.method == 'GET':
        return jsonify({'aux': fp.aux})
    fp.aux = parse_bool(request.data)
    return jsonify({'aux': fp.aux})


@app.route("/front", methods=['GET', 'PUT'])
def front():
    if request.method == 'GET':
        return jsonify({'front': fp.front})
    fp.front = parse_bool(request.data)
    return jsonify({'front': fp.front})


# ── GPIO routes ───────────────────────────────────────────────────────────────

@app.route("/gpio/<int:pin>/mode", methods=['GET', 'PUT'])
def gpio_mode(pin):
    ps = _pin_state(pin)
    if request.method == 'GET':
        return jsonify({'pin': pin, 'mode': ps['mode']})

    mode = request.data.decode().strip().lower()
    if mode not in ('input', 'output', 'pwm'):
        abort(400, "mode must be input, output, or pwm")

    _cleanup_pwm(ps)
    pull_map = {'up': GPIO.PUD_UP, 'down': GPIO.PUD_DOWN, 'off': GPIO.PUD_OFF}
    pull = pull_map.get(ps['pull'], GPIO.PUD_OFF)

    if mode == 'input':
        GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
    elif mode in ('output', 'pwm'):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        ps['state'] = False

    ps['mode'] = mode
    return jsonify({'pin': pin, 'mode': ps['mode']})


@app.route("/gpio/<int:pin>/pull", methods=['GET', 'PUT'])
def gpio_pull(pin):
    ps = _pin_state(pin)
    if request.method == 'GET':
        return jsonify({'pin': pin, 'pull': ps['pull']})

    if ps['mode'] != 'input':
        abort(400, f"Pull resistor only applies in input mode (currently: {ps['mode']})")

    pull = request.data.decode().strip().lower()
    if pull not in ('up', 'down', 'off'):
        abort(400, "pull must be up, down, or off")

    pull_map = {'up': GPIO.PUD_UP, 'down': GPIO.PUD_DOWN, 'off': GPIO.PUD_OFF}
    GPIO.setup(pin, GPIO.IN, pull_up_down=pull_map[pull])
    ps['pull'] = pull
    return jsonify({'pin': pin, 'pull': ps['pull']})


@app.route("/gpio/<int:pin>/state", methods=['GET', 'PUT'])
def gpio_state(pin):
    ps = _pin_state(pin)
    if ps['mode'] != 'output':
        abort(400, f"Pin must be in output mode to get/set state (currently: {ps['mode']})")

    if request.method == 'GET':
        return jsonify({'pin': pin, 'state': ps['state']})

    value = parse_bool(request.data)
    GPIO.output(pin, GPIO.HIGH if value else GPIO.LOW)
    ps['state'] = value
    return jsonify({'pin': pin, 'state': ps['state']})


@app.route("/gpio/<int:pin>/reading", methods=['GET'])
def gpio_reading(pin):
    _pin_state(pin)
    return jsonify({'pin': pin, 'reading': bool(GPIO.input(pin))})


@app.route("/gpio/<int:pin>/pwm", methods=['GET', 'PUT', 'DELETE'])
def gpio_pwm(pin):
    ps = _pin_state(pin)
    if ps['mode'] != 'pwm':
        abort(400, f"Pin must be in pwm mode (currently: {ps['mode']})")

    if request.method == 'GET':
        return jsonify({
            'pin':        pin,
            'active':     ps['pwm'] is not None,
            'frequency':  ps['pwm_freq'],
            'duty_cycle': ps['pwm_duty'],
        })

    if request.method == 'DELETE':
        _cleanup_pwm(ps)
        return jsonify({'pin': pin, 'active': False})

    data = request.get_json(force=True)
    freq = float(data.get('frequency', 1000))
    duty = float(data.get('duty_cycle', 50))

    if not (0 < freq <= 1_000_000):
        abort(400, "frequency must be between 1 and 1000000 Hz")
    if not (0 <= duty <= 100):
        abort(400, "duty_cycle must be between 0 and 100")

    _cleanup_pwm(ps)
    pwm = GPIO.PWM(pin, freq)
    pwm.start(duty)
    ps['pwm']      = pwm
    ps['pwm_freq'] = freq
    ps['pwm_duty'] = duty

    return jsonify({'pin': pin, 'active': True, 'frequency': freq, 'duty_cycle': duty})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        for ps in _gpio_pins.values():
            _cleanup_pwm(ps)
        fp.cleanup()
