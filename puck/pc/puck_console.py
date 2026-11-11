#!/usr/bin/env python3
"""Talk to the Nucleo through the DK1 (or directly over its ST-LINK COM port).

  python puck_console.py socket://192.168.1.238:4000          # live view + type commands
  python puck_console.py COM4 --cmd PING --cmd SCAN --secs 3  # send commands, print replies
Commands: PING, SCAN, PROBE, STREAM ON|OFF, ESP RESET, ESP BOOT, ESP SEND <text>, STATS
"""
import argparse, sys, threading, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("port"); ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--cmd", action="append", default=[]); ap.add_argument("--secs", type=float, default=0)
a = ap.parse_args()
s = serial.serial_for_url(a.port, baudrate=a.baud, timeout=0.2)
def reader():
    while True:
        d = s.read(4096)
        if d: sys.stdout.write(d.decode(errors="replace")); sys.stdout.flush()
threading.Thread(target=reader, daemon=True).start()
for c in a.cmd:
    s.write((c + "\n").encode()); time.sleep(0.3)
if a.cmd or a.secs:
    time.sleep(a.secs or 1.5); sys.exit(0)
for line in sys.stdin:
    s.write(line.rstrip("\r\n").encode() + b"\n")
