# pixhawk-drone — F450 quadcopter on a Pixhawk PRO, flown from a laptop

Companion repository of the project page: **https://how2222.github.io/portfolio/projects/drone.html**

A 450-class quadcopter built from parts between January and April 2026: F450 frame, Pixhawk PRO running
ArduCopter 4.6.3 (fmuv3 build), A2212 motors with 30 A ESCs, a 3S 3000 mAh LiPo, a Raspberry Pi 4 as
companion computer, and no RC transmitter — the drone is flown from a MacBook over a pair of SiK 433 MHz
telemetry radios with the program in this repository.

## Contents

| File | What it is |
|---|---|
| `mavcontrol.py` | Keyboard ground-control program (Python + pygame + pymavlink): arm / disarm, take-off, land, throttle, flight modes, AHRS trim, live telemetry monitor and a small command line (`motortest`, `set`, `mode`, `reboot`). |
| `raspberry-pi.md` | How the Raspberry Pi 4 is linked to the flight controller over TELEM 2 (UART, MAVLink 2, MAVProxy). |
| `requirements.txt` | Python dependencies of `mavcontrol.py`. |
| `params/` | The exported ArduCopter parameter file — not exported yet. |

## Running the ground-control program

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
python3 mavcontrol.py --port /dev/tty.usbserial-0001 --baudrate 57600
```

`--port` is the SiK ground radio plugged into the laptop (57 600 baud by default). Keys are listed in the
window; `Space` is the emergency kill (force disarm). `M` opens the telemetry monitor, where `Tab` gives a
command line.

![mavcontrol.py](https://how2222.github.io/portfolio/assets/drone/mavcontrol-ui.png)

## Flight-controller setup (ArduCopter 4.6.3, Quad X)

- `SERIAL2_PROTOCOL = 2`, `SERIAL2_BAUD = 57` — TELEM 2 to the Raspberry Pi
- `LOG_BACKEND_TYPE = 3` — dataflash logs also streamed over MAVLink
- RC and GPS arming checks disabled, `FS_THR_ENABLE = 0`, `DISARM_DELAY = 0` (no RC transmitter, flown indoors)
- Outputs as wired: 1 → front-right (M1, CCW), 2 → front-left (M3, CW), 3 → rear-right (M4, CW), 4 → rear-left (M2, CCW) — `SERVOn_FUNCTION` must match

## Status

The drone lifts off indoors (Stabilize and Alt Hold) but yaws on its own as soon as it leaves the ground.
The history of the problem and the open hypotheses are on the project page.
