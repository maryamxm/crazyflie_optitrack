# `mocap.py`: motion capture helper

`mocap.py` is the shared module that connects OptiTrack Motive to the Crazyflies. Every flight script (`go_to.py`, `swarm_hover.py`, `swarm_square.py`, `swarm_status.py`) uses it, so all the Motive/NatNet handling lives in one place.

## What it does

1. Receives rigid-body poses from Motive over NatNet.
2. Keeps the latest pose of each drone.
3. Forwards each pose to its drone (extpose: position + orientation) so the onboard Kalman filter can use it.
4. Detects lost or frozen tracking, so flight scripts can land a drone safely.
5. Prepares each drone's estimator for mocap (reset_estimator).

## Setup: Motive streaming settings

* Transmission Type: **Unicast**
* Up Axis: **Z Up**
* Broadcast Frame Data: **On**
* One rigid body per drone, each with a **unique Streaming ID**

## How it works

1. `Mocap.start()` starts a NatNet client. NatNet receives frames from Motive on its own background thread.
2. For each rigid body in a frame, the Streaming ID is matched to a drone.
3. `DroneState.update()` stores the pose and timestamp, and rejects frozen data.
4. If forwarding is on for that drone, the pose is sent immediately with `cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)`.
5. Flight scripts read the latest pose and check tracking from their own threads.

**Threading:** the NatNet thread writes and the flight/swarm threads read. 
**Frames:** poses are passed through unchanged, in the Motive world frame (metres, Z up). The Crazyflie's estimate and every `go_to(..., relative=False)` target are therefore in Motive coordinates too.

## Tuning parameters

| **Parameter** | **Default** | **Raise it if…** | **Lower it if…** |
|---|---|---|---|
| tracking_timeout | 0.3 s | You get false TrackingLost from short network hiccups | You want the drone to land sooner after tracking loss |
| stale_frames | 10 | A drone sitting perfectly still on the ground triggers it | Frozen data is forwarded for too long before being caught |
| orientation_std_dev | 8.0e-3 | Yaw is jittery because the mocap orientation is noisy | Yaw drifts or responds slowly |

> NOTE: At Motive's frame rate (e.g. 120 Hz), stale_frames=10 is about 80 ms of identical data.

## Notes

> WARNING: Tracking loss detection: currently the frozen frames are detected correctly, however the tracking loss does not work yet. 
> The SDK's `NatNetClient.py` does not pass Motive's `per-rigid-body tracking_valid` flag to the rigid-body listener. If `mocap.py` uses `rigid_body_listener`, its `tracking_valid` check never takes effect, and only the timeout and frozen-frame checks protect the drone. 

> TIP: Comment out line 2082 in `NatNetClient.py`. The line reads as `print("MoCap Frame: %d\n" % (mocap_data.prefix_data.frame_number))`, which prints the frame number of each streamed pose and orientation, which results in clutter in the terminal and will make debugging hard.

