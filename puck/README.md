# Puck dev-kit prototype software

Functional prototype: NUCLEO-H753ZI + STM32MP157D-DK1 + ESP32-WROOM-32D.
Wiring: H753 UART7 (PE8/PE7) <-> DK1 Arduino D0/D1 (`/dev/ttySTM2`);
H753 USART6 (PC6/PC7) <-> ESP32 UART0 (GPIO3/GPIO1); H753 PE9 -> ESP32 EN, PE11 -> ESP32 GPIO0.

| Folder | Runs on | What |
|---|---|---|
| `dk1/` | DK1 (Linux) | `run.sh` app entry point, `tcp_serial.py` (Nucleo link on TCP 4000), auto-update scripts |
| `h753-fw/` | Nucleo H753 | `puck_h7` firmware source (`make`), prebuilt `puck_h7.bin`/`.elf` |
| `pc/` | PC | `puck_console.py` (talk to the Nucleo), `esp_via_h7.py` (esptool through the H753) |

## Auto-update on the DK1

The DK1 already follows this repo: `repo-sync.timer` fetches `main` every minute into
`/home/root/mp157Repo`, resets to it, and restarts `mp157-app.service`. That launcher builds and runs
`orange/` if `orange/main.c` compiles, otherwise runs `main.py` at the repo root, which starts
`puck/dk1/run.sh`. So: push to `main` and the puck runs it within a minute.

Useful on the DK1: `journalctl -u repo-sync -n 20`, `journalctl -u mp157-app -f`, `systemctl restart mp157-app`.

## Nucleo firmware

Flash from the PC (ST-LINK on the Nucleo):

    STM32_Programmer_CLI -c port=SWD -w puck/h753-fw/puck_h7.elf -v -rst

Text commands on the DK1 link or the ST-LINK COM port: `PING`, `SCAN`, `STREAM ON|OFF`,
`ESP RESET`, `ESP BOOT`, `ESP SEND <text>`, `BRIDGE` (esptool pass-through), `STATS`.

## ESP32 through the H753

    python puck/pc/esp_via_h7.py socket://<dk1-ip>:4000 chip-id

## ST drivers (not in the repo)

`puck/h753-fw/puck_h7/drivers/` (HAL + CMSIS headers/sources) is not committed. It is fetched by
`install_puck_sw.py` from the pinned STMicroelectronics repos: stm32h7xx_hal_driver @ 7e541d9,
cmsis_device_h7 @ 81db1ec, cmsis_core @ 01d1814. That script also copies `system_stm32h7xx.c` and
`startup_stm32h753xx.s` into `src/`.
