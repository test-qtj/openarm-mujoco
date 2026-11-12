#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/fn/miniconda3/envs/lerobot-hf/bin/python}"
VT13_PORT="${VT13_PORT:-/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00}"
MQTT_PORT="${SIM_MQTT_PORT:-1885}"
CLIENT_ID="${SIM_MQTT_CLIENT_ID:-sim-robot}"

for tool in socat mosquitto; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Missing $tool. Install it with: sudo apt install $tool" >&2
        exit 1
    fi
done

if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "Python environment not found: $PYTHON_BIN" >&2
    exit 1
fi
if ! "$PYTHON_BIN" -c 'import serial, paho.mqtt.client' >/dev/null 2>&1; then
    echo "Install project dependencies first:" >&2
    echo "  $PYTHON_BIN -m pip install -e '$REPO_DIR[vt13,competition]'" >&2
    exit 1
fi
if [[ ! -e "$VT13_PORT" ]]; then
    echo "VT13 serial device not found: $VT13_PORT" >&2
    echo "Set VT13_PORT to the current /dev/serial/by-id/... path and retry." >&2
    exit 1
fi

SOCAT_LOG="$(mktemp)"
BROKER_LOG="$(mktemp)"
SOCAT_PID=""
BROKER_PID=""
SIM_PID=""

port_open() {
    "$PYTHON_BIN" -c 'import socket,sys; s=socket.socket(); s.settimeout(0.2); s.connect(("127.0.0.1", int(sys.argv[1]))); s.close()' "$MQTT_PORT" >/dev/null 2>&1
}

cleanup() {
    local result=$?
    trap - EXIT INT TERM
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill -INT "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
    if [[ -n "$SOCAT_PID" ]] && kill -0 "$SOCAT_PID" 2>/dev/null; then
        kill "$SOCAT_PID" 2>/dev/null || true
        wait "$SOCAT_PID" 2>/dev/null || true
    fi
    if [[ -n "$BROKER_PID" ]] && kill -0 "$BROKER_PID" 2>/dev/null; then
        kill "$BROKER_PID" 2>/dev/null || true
        wait "$BROKER_PID" 2>/dev/null || true
    fi
    rm -f "$SOCAT_LOG" "$BROKER_LOG"
    exit "$result"
}
trap cleanup EXIT INT TERM

if port_open; then
    echo "Using existing local MQTT broker on port $MQTT_PORT."
else
    mosquitto -p "$MQTT_PORT" >"$BROKER_LOG" 2>&1 &
    BROKER_PID=$!
    for _ in $(seq 1 50); do
        if port_open; then
            break
        fi
        if ! kill -0 "$BROKER_PID" 2>/dev/null; then
            cat "$BROKER_LOG" >&2
            exit 1
        fi
        sleep 0.1
    done
    if ! port_open; then
        cat "$BROKER_LOG" >&2
        echo "MQTT broker did not start on port $MQTT_PORT." >&2
        exit 1
    fi
fi

socat -d -d pty,raw,echo=0 pty,raw,echo=0 2>"$SOCAT_LOG" &
SOCAT_PID=$!
PTY_PATHS=()
for _ in $(seq 1 50); do
    mapfile -t PTY_PATHS < <(sed -n 's/.*PTY is //p' "$SOCAT_LOG")
    if [[ ${#PTY_PATHS[@]} -ge 2 ]]; then
        break
    fi
    if ! kill -0 "$SOCAT_PID" 2>/dev/null; then
        cat "$SOCAT_LOG" >&2
        exit 1
    fi
    sleep 0.1
done
if [[ ${#PTY_PATHS[@]} -lt 2 ]]; then
    cat "$SOCAT_LOG" >&2
    echo "Could not create a virtual serial pair." >&2
    exit 1
fi

SERIAL_TX="${PTY_PATHS[0]}"
SERIAL_RX="${PTY_PATHS[1]}"
echo "VT13: $VT13_PORT"
echo "Local MQTT: 127.0.0.1:$MQTT_PORT ($CLIENT_ID)"
echo "Virtual serial: TX $SERIAL_TX → RX $SERIAL_RX"
echo "Starting paired MuJoCo simulation; press Ctrl+C to stop."

cd "$REPO_DIR"
"$PYTHON_BIN" -u v1/master_follower_sim.py --fps 60 \
    --vt13-port "$VT13_PORT" \
    --serial-tx "$SERIAL_TX" --serial-rx "$SERIAL_RX" \
    --mqtt-host 127.0.0.1 --mqtt-port "$MQTT_PORT" \
    --mqtt-client-id "$CLIENT_ID" --mqtt-bridge-to-serial &
SIM_PID=$!
wait "$SIM_PID"
SIM_PID=""
