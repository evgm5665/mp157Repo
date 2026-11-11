#!/bin/sh
# Puck app entry point, started by mp157-app.service and restarted on every code update.
# v0.2: live camera on http://<dk1-ip>:8080/ (templates/mjpeg_stream.c) and the Nucleo
# link shared on TCP port 4000. Both stop together when the service restarts.
cd "$(dirname "$0")/.."
BUILD=/home/root/mp157-build
mkdir -p "$BUILD"
if zig cc -O2 -o "$BUILD/mjpeg_stream" ../templates/mjpeg_stream.c; then
    echo "camera: http://$(hostname -I 2>/dev/null | cut -d' ' -f1):8080/"
    "$BUILD/mjpeg_stream" auto 8080 &
else
    echo "camera: mjpeg_stream failed to build, running without it"
fi
exec python3 -u dk1/tcp_serial.py --dev /dev/ttySTM2 --port 4000 --baud 115200
