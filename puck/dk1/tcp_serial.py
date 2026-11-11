#!/usr/bin/env python3
"""Share the Nucleo link (/dev/ttySTM2, UART7 on Arduino D0/D1) over TCP.

Run on the DK1:  python3 tcp_serial.py [--dev /dev/ttySTM2] [--port 4000] [--baud 115200]
Then from the PC: esptool / pyserial with  socket://<dk1-ip>:4000
One client at a time; a new client replaces the old one. Raw bytes, no framing.
Uses only the standard library (termios), no pyserial needed.
"""
import argparse, os, select, socket, termios, tty

BAUDS = {9600: termios.B9600, 115200: termios.B115200, 230400: termios.B230400,
         460800: termios.B460800, 921600: termios.B921600}

def open_serial(dev, baud):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = BAUDS[baud]
    a[2] |= termios.CLOCAL | termios.CREAD
    a[2] &= ~(termios.CRTSCTS | termios.PARENB | termios.CSTOPB)
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttySTM2")
    ap.add_argument("--port", type=int, default=4000)
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()
    fd = open_serial(a.dev, a.baud)
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", a.port)); srv.listen(1)
    print(f"sharing {a.dev} @ {a.baud} on tcp:{a.port}", flush=True)
    cli = None
    while True:
        rl = [fd, srv] + ([cli] if cli else [])
        r, _, _ = select.select(rl, [], [], 1.0)
        if srv in r:
            if cli: cli.close()
            cli, addr = srv.accept()
            cli.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("client", addr, flush=True)
        if fd in r:
            try: data = os.read(fd, 4096)
            except BlockingIOError: data = b""
            if data and cli:
                try: cli.sendall(data)
                except OSError: cli.close(); cli = None
        if cli and cli in r:
            try: data = cli.recv(4096)
            except OSError: data = b""
            if not data:
                cli.close(); cli = None
            else:
                while data:
                    try: n = os.write(fd, data); data = data[n:]
                    except BlockingIOError: select.select([], [fd], [], 0.1)

if __name__ == "__main__":
    main()
