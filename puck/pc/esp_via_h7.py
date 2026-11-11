#!/usr/bin/env python3
"""Run esptool on the ESP32 through the Nucleo H753 (which drives EN/GPIO0 and bridges UART0).

  python esp_via_h7.py socket://192.168.1.238:4000 chip-id
  python esp_via_h7.py socket://192.168.1.238:4000 write-flash 0x0 app_merged.bin
  python esp_via_h7.py COM4 chip-id              # same, but through the Nucleo's ST-LINK port
Everything after the port is passed to esptool. The H753 bridge runs at 115200 and ends
after 3 s of silence, so esptool runs with --baud 115200 and no reset of its own.
"""
import subprocess, sys, time
import serial

port, args = sys.argv[1], sys.argv[2:]
s = serial.serial_for_url(port, baudrate=115200, timeout=0.2)
s.reset_input_buffer()
s.write(b"STREAM OFF\nBRIDGE\n")
t0, buf = time.time(), b""
while time.time() - t0 < 3 and b"OK BRIDGE BOOT" not in buf:
    buf += s.read(256)
s.close()
if b"OK BRIDGE BOOT" not in buf:
    sys.exit("Nucleo did not answer BRIDGE; got: %r" % buf[-200:])
cmd = [sys.executable, "-m", "esptool", "--port", port, "--baud", "115200",
       "--before", "no-reset", "--after", "no-reset"] + args
print(" ".join(cmd), flush=True)
rc = subprocess.call(cmd)
time.sleep(3.5)   # let the bridge time out; the H753 then resets the ESP32 into its app
s = serial.serial_for_url(port, baudrate=115200, timeout=0.5)
s.write(b"STREAM ON\nPING\n"); time.sleep(1); print(s.read(2000).decode(errors="replace"))
sys.exit(rc)
