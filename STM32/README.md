# STM32H7 OTA Bootloader

A bootloader for the **STM32H743**, with **OTA over-the-air updates + health check + automatic rollback**.
It receives new firmware over Ethernet; if the new version is broken, it automatically restores the last working one.

> **⚠️ Currently under development.**

## Features

- **OTA updates over Ethernet**
  - Press KEY1 at boot to open the menu, then select [2] to download and write new firmware from the PC
  - Automatic **CRC verification** before writing — a bad image is rejected
  - New firmware is written to Bank 2 (`0x08100000`), separate from the bootloader
- **Health check + automatic rollback**
  - After an update, the new App enters a "trial period"; if it hangs, the watchdog resets the board after 5 seconds
  - On reboot the bootloader detects the failure ➜ restores the previous working firmware (no brick risk)
- **Interactive menu** (USART 115200)
  - [1] Info (incl. debug info), [2] Update, [3] Backup, [4] Restore, [5] Run, [6] Reboot, [9] QSPI test
- **Manual backup / restore**
  - One-step backup of the current firmware to external flash, restorable at any time

## Directory Structure

```
STM32/
├── Bootloader/    Complete Keil project
└── README*.md
```

## Development Environment

- Keil μVision + STM32H7xx HAL + LwIP 2.0.3 (bare-metal Raw API)
- PC tools: Python 3.11