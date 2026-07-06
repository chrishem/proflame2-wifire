#!/usr/bin/env python3
"""
server.py: Flask REST server for the Proflame 2 fireplace controller.

Endpoints:
  GET  /state              -- full fireplace state as JSON
  PUT  /state              -- set multiple params at once (JSON body)
  GET  /power              -- current power state
  PUT  /power              -- set power (body: true/false)
  GET  /flame              -- current flame level
  PUT  /flame              -- set flame level (body: 0-6)
  GET  /fan                -- current fan level
  PUT  /fan                -- set fan level (body: 0-6)
  GET  /light              -- current light level
  PUT  /light              -- set light level (body: 0-6)
  GET  /pilot              -- current pilot state
  PUT  /pilot              -- set pilot (body: true/false)
  GET  /thermostat         -- current thermostat state
  PUT  /thermostat         -- set thermostat (body: true/false)
  GET  /aux                -- current aux state
  PUT  /aux                -- set aux (body: true/false)
  GET  /front              -- current front flame state
  PUT  /front              -- set front flame (body: true/false)
  GET  /serial             -- current serial number
"""

import logging
from flask import Flask, request, jsonify, abort
from fireplace import Fireplace

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
fp  = Fireplace()


# ── Helpers ───────────────────────────────────────────────────────────────────

def parse_bool(data):
    """Parse request body as bool. Accepts 'true'/'false' or '1'/'0'."""
    s = data.decode().strip().lower()
    if s in ('true', '1'):
        return True
    if s in ('false', '0'):
        return False
    abort(400, f"Expected true/false, got: {s}")


def parse_int(data, lo=0, hi=6):
    """Parse request body as int within [lo, hi]."""
    try:
        v = int(data.decode().strip())
    except ValueError:
        abort(400, f"Expected integer {lo}–{hi}")
    if not (lo <= v <= hi):
        abort(400, f"Value {v} out of range {lo}–{hi}")
    return v


# ── Routes ────────────────────────────────────────────────────────────────────

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


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        fp.cleanup()
