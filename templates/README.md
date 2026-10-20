# Templates

Starting points to copy into `orange/` (the board auto-builds and runs `orange/*.c` on every push).
Files here are **not** built automatically.

| File | What it does |
|---|---|
| `usb_camera.c` | Grabs frames from a USB webcam (`/dev/video0`) with V4L2, prints fps, saves the last frame |
| `net_packets.c` | Sends/receives UDP packets, pinned to `eth0`, `wlan0` or any interface |
| `opencv_camera.cpp` | Reads the camera with OpenCV and runs edge detection (C++, see notes) |
| `braid_client.c` | Receives live 3D tracking from Strand-Braid and optionally forwards it over UDP |

## Using one

Copy it in as the app and push:

```sh
cp templates/usb_camera.c orange/main.c
git add orange/main.c && git commit -m "use camera template" && git push
```

Only one file in `orange/` may contain `main()`, since every `.c` there is compiled into one program.

To try one by hand on the board instead:

```sh
cd /home/root/mp157Repo/templates
zig cc -O2 usb_camera.c -o /tmp/usb_camera && /tmp/usb_camera
```

## Notes

- **Wi-Fi:** the DK1 has no built-in Wi-Fi (the DK2 does). `wlan0` needs a USB Wi-Fi adapter whose driver is in the kernel.
- **OpenCV:** OpenCV 4 is C++ only. ST's package feed has the OpenCV 4.5 libraries, but the board has no C++ headers or g++ yet. The easiest route is Python: `apt-get install python3-opencv`, then `import cv2`.
- **Braid:** runs on the tracking PC, not the board. Use the URL it prints at startup (`http://<host>:<port>/?token=...`) and make sure Braid listens on an address the board can reach.
