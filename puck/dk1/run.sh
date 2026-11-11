#!/bin/sh
# Puck app entry point, started by puck.service and restarted on every code update.
# v0: share the Nucleo link over TCP port 4000. The camera/detector app gets added here next.
cd "$(dirname "$0")/.."
exec python3 dk1/tcp_serial.py --dev /dev/ttySTM2 --port 4000 --baud 115200
