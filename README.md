# Crazyflie + OptiTrack Swarm Control

Python scripts for flying one or more Crazyflie drones using OptiTrack (Motive) motion capture as the position source. Motive streams each drone's pose over NatNet. The scripts forward it to the drone's onboard controller and run single-drone and swarm flights.

<!-- Add a demo GIF or video link here -->
<!-- ![Demo](docs/demo.gif) -->

![Lab setup with 4 Crazyflies](docs/media/setup.jpg)
![Swarm flying the square pattern](media/squareFly.gif)

[![Watch the flight demo](docs/media/video_thumbnail.jpg)](https://drive.google.com/file/d/FILE_ID/view?usp=sharing)

---

## Features

- Streams rigid-body poses from Motive to each Crazyflie (`extpose`: position + orientation)
- Detects lost or frozen tracking and lands the affected drone(s)
- Refuses to fly if a waypoint is outside a configurable safe flight box
- Every drone lands on Ctrl-C or on any error
- Swarm status monitor: battery, mocap pose, onboard estimate, estimate error, Kalman uncertainty, supervisor flags
- Swarm power control: soft off/on, shutdown, reboot

---

## Repository structure

```
crazyflie-optitrack/
├── requirements.txt
└── mocap_crazyflie/
    ├── mocap.py           # Shared NatNet/Motive handling, pose forwarding, tracking checks
    ├── drone_pose.py      # Print one drone's pose from Motive (no drone connection)
    ├── go_to.py           # Fly one drone to a target point, hover, land
    ├── swarm_hover.py     # Swarm takeoff/hover/land (sequential or parallel), optional square
    ├── swarm_square.py    # 4-drone pattern in absolute Motive coordinates
    ├── swarm_status.py    # Live status monitor for the swarm
    ├── cf_power.py        # Power on/off/reboot multiple drones over Crazyradio
    └── PythonClient/      # NatNet SDK files go here (not included, see below)
```

---

## Hardware

| Item | Used / tested |
|---|---|
| Drone | Crazyflie <2.1 / 2.1+ / Brushless> |
| Firmware | <e.g. 2025.xx> |
| Radio | Crazyradio <PA / 2.0> |
| Markers | <e.g. 4 passive markers per drone, unique layout per drone> |
| Mocap | OptiTrack <camera model, number of cameras> |
| Motive version | <e.g. 3.x> |
| NatNet SDK version | <e.g. 4.x> |

---

## Software requirements

- Python 3.12 (tested with 3.12.15 on <macOS / Ubuntu xx.04>)
- [cflib](https://github.com/bitcraze/crazyflie-lib-python) 0.1.34
- OptiTrack NatNet SDK (Python client)

---

## Installation

### 1. Clone and set up a virtual environment

```bash
git clone https://github.com/<your-username>/crazyflie-optitrack.git
cd crazyflie-optitrack
python3.12 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Crazyradio USB permissions (Linux only)

Follow Bitcraze's USB permission guide so the Crazyradio works without `sudo`:
<link to Bitcraze docs>

### 3. Add the NatNet SDK files

These files are **not included** in this repository because NaturalPoint's licence does not allow redistributing them.

1. Download the NatNet SDK from OptiTrack: <https://optitrack.com/support/downloads/developer-tools.html>
2. From the SDK's Python sample folder, copy these files into `mocap_crazyflie/PythonClient/`:
   - `NatNetClient.py`
   - `DataDescriptions.py`
   - `MoCapData.py`

---

## Motive setup

In Motive:

1. **Rigid bodies:** create one rigid body per drone and give each a **unique Streaming ID**.
2. **Orientation:** align each rigid body's orientation with the drone's body frame (X forward). Orientation is sent to the Kalman filter, so a wrong pivot or rotation will cause bad flight.
3. **Streaming settings (Data Streaming pane):**
   - Broadcast Frame Data: **On**
   - Transmission Type: **Unicast**
   - Up Axis: **Z Up**
   - Local Interface: <the Motive PC's IP on the shared network>
4. **Ground plane:** set it so the origin and axes match the flight-box limits you configure below.

---

## Network setup

- The laptop running these scripts and the Motive PC must be on the **same network / subnet**.
- The firewall on both machines must allow **UDP ports 1510 and 1511** (NatNet).
- `CLIENT_IP` = this laptop's IP, `SERVER_IP` = the Motive PC's IP.

<!-- Add notes about your lab network, e.g. wired vs Wi-Fi, if relevant -->

---

## Configuration

Each script has a **SETTINGS** section at the top. Edit:

| Setting | Meaning |
|---|---|
| `DRONE_CONFIG` | `{ radio URI : Motive Streaming ID }`, one entry per drone. Order = flight order where relevant |
| `URI` / `DRONE_ID` | Single-drone scripts (`go_to.py`, `drone_pose.py`) |
| `CLIENT_IP`, `SERVER_IP` | See Network setup |
| `X_LIMITS`, `Y_LIMITS`, `Z_LIMITS` | Safe flight box in Motive coordinates (m). Keep these well inside the tracked volume |
| `TAKEOFF_HEIGHT`, `FLIGHT_HEIGHT`, speeds | Flight parameters |

URI format: `radio://<radio number>/<channel>/<data rate>/<address>`, e.g. `radio://0/80/2M/E7E7E7E7E1`.
Each drone needs a unique address. Set addresses with the Crazyflie client (cfclient).

`cf_power.py` uses its own `URIS` list.

---

## Usage

All commands run from `mocap_crazyflie/` with the virtual environment active.

### Recommended order for a first session

```bash
# 1. Wake the drones
python3 cf_power.py on

# 2. Check batteries (Motive not needed)
python3 swarm_status.py --show battery

# 3. Check Motive is streaming the right rigid body
python3 drone_pose.py

# 4. Check mocap vs onboard estimate
python3 swarm_status.py --show pose

# 5. Single-drone flight
python3 go_to.py

# 6. Swarm hover
python3 swarm_hover.py

# 7. Swarm pattern
python3 swarm_square.py
```

### Scripts

| Script | What it does | Needs Motive |
|---|---|---|
| `drone_pose.py` | Prints x, y, z and quaternion for one rigid body | Yes |
| `go_to.py` | Takeoff → fly to target → hover → land (single drone) | Yes |
| `swarm_hover.py` | LED check → estimator reset → arm → hover → land. Switch between sequential and parallel in `__main__` | Yes |
| `swarm_square.py` | Drones fly from anywhere to their start points (staggered transit heights), then run the pattern in parallel. Requires exactly 4 drones | Yes |
| `swarm_status.py` | Status snapshot or live view (`--watch`) | Only for `mocap`, `estimate`, `error`, `kalman` |
| `cf_power.py` | `on`, `off`, `shutdown`, `reboot`, `cycle`; target with `-d` (indices) or `-u` (URIs) | No |

`swarm_status.py` examples:

```bash
python3 swarm_status.py                          # everything, one snapshot
python3 swarm_status.py --show pose supervisor   # combine sections
python3 swarm_status.py --show battery --watch   # live view, Ctrl-C to stop
```

---

## How it works

<!-- Optional: add a diagram -->

1. `mocap.py` runs a NatNet client that receives rigid-body poses from Motive.
2. Each pose is matched to a drone by Streaming ID and sent to that drone with `cf.extpos.send_extpose()`.
3. The Kalman estimator is selected and reset, and the script waits until its position variance settles.
4. Flights use the Crazyflie high-level commander (`takeoff`, `go_to`, `land`).
5. During flight, `safe_sleep()` checks tracking continuously and raises `TrackingLost` if a drone is not seen for `TRACKING_TIMEOUT` or its pose freezes for `STALE_FRAMES` frames.

---

## Safety

- **Test with one drone first**, inside a net if possible.
- Set the flight-box limits for **your** space. The defaults are for <your lab>.
- **Tracking loss** lands the affected drone. In swarm scripts, an error lands all drones.
- **Ctrl-C** lands all drones.
- Check batteries before flying. `swarm_status.py` flags resting voltage below 3.5 V.
- Keep drones at least ~0.3 m apart on the floor.
- <Add your lab's own rules>

---

## Known limitations

- `Z_LIMITS` <has not been fully verified for the space>
- `swarm_square.py` is hardcoded for 4 drones
- Every Motive frame is forwarded to every drone. With several drones on one Crazyradio, a high Motive frame rate may congest the radio link
- <Add others>

---

## Troubleshooting

| Problem | Likely cause |
|---|---|
| `No valid mocap data for: ...` | Wrong Streaming ID, streaming off, wrong IPs, firewall blocking UDP 1510/1511 |
| `Estimator did not converge` | Mocap poses not reaching the drone, rigid body orientation wrong |
| Drone flips or drifts on takeoff | Rigid body orientation does not match the drone frame |
| Cannot connect to drone | Wrong URI/address, Crazyradio permissions, drone off |
| <add your own> | |

---

## Acknowledgements

- [Bitcraze](https://www.bitcraze.io/): cflib and its examples, which the estimator reset and swarm structure are based on
- [OptiTrack / NaturalPoint](https://optitrack.com/): NatNet SDK
- <Lab / supervisor / university>

---

## License

<Choose a licence, e.g. MIT, and add a LICENSE file. Check cflib's licence before you choose.>

## Author

Maryam Mosadegh: <GitHub / LinkedIn / email>
