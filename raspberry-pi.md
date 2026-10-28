# Raspberry Pi 4 ↔ Pixhawk PRO link

Procedure followed from the ArduPilot wiki: https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html

## Wiring (TELEM 2, crosswise)

| Pixhawk TELEM 2 | Raspberry Pi 4 header |
|---|---|
| 5 V | pin 2 (5 V) — the Pi is powered by the flight controller's 5 V rail |
| TX | pin 10 (GPIO 15, RXD) |
| RX | pin 8 (GPIO 14, TXD) |
| GND | pin 6 (GND) |

Flight-controller parameters: `SERIAL2_PROTOCOL = 2` (MAVLink 2), `SERIAL2_BAUD = 57` (57 600 baud),
`LOG_BACKEND_TYPE = 3`.

## 1. Enable the UART on the Pi

```
sudo raspi-config  →  Interfacing Options  →  Serial
  Would you like a login shell to be accessible over serial?   No
  Would you like the serial port hardware to be enabled?       Yes
sudo reboot        # the UART is now /dev/serial0
```

## 2. Packages and a virtual environment

```bash
sudo apt-get update
sudo apt-get install python3-wxgtk4.0 python3-matplotlib python3-opencv python3-pip \
                     python3-numpy libxml2-dev libxslt1-dev python3-dev
sudo apt install python3-venv python3-full
mkdir ~/mavlink_project && cd ~/mavlink_project
python3 -m venv venv
source venv/bin/activate
pip install pymavlink
pip install mavproxy
```

## 3. Start the link (by hand, over SSH)

```bash
cd ~/mavlink_project && source venv/bin/activate
mavproxy.py --master=/dev/serial0 --baudrate 57600 --aircraft MyCopter
```
