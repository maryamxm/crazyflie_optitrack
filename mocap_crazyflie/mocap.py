"""
Mocap helper for Crazyflie swarms with OptiTrack Motive (NatNet).

Receives rigid-body poses from Motive, keeps the latest pose per drone,
forwards poses to the drones, and detects lost or frozen tracking.

Usage:
    from mocap import Mocap, TrackingLost, reset_estimator

    with Mocap(DRONE_CONFIG, CLIENT_IP, SERVER_IP) as mocap:
        mocap.wait_for_all()                    # every drone visible?
        with Swarm(mocap.uris, factory=factory) as swarm:
            swarm.parallel_safe(mocap.start_forwarding)
            swarm.parallel_safe(reset_estimator)
            ...
            drone = mocap.drone_for(scf)        # inside per-drone functions
            drone.get_pose(), drone.safe_sleep(2), drone.wait_until_at((x, y, z))

Requirements:
    - Motive streaming: Unicast, Up Axis = Z Up, Broadcast Frame Data ON
    - One rigid body per drone in Motive, each with a unique Streaming ID
    - NatNetClient.py in a 'PythonClient' folder next to this file
"""

import math
import os
import sys
import threading
import time

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'PythonClient'))
from NatNetClient import NatNetClient


class TrackingLost(Exception):
    pass


# ----- One drone

class DroneState:
    """Latest pose of one drone, when it was last seen, and the link to forward
    poses to. Shared between the NatNet thread (writes) and swarm threads (read)."""

    def __init__(self, uri, rb_id, tracking_timeout, stale_frames):
        self.uri = uri
        self.rb_id = rb_id
        self.tracking_timeout = tracking_timeout
        self.stale_frames = stale_frames
        self.lock = threading.Lock()
        self.pose = None              # (x, y, z, qx, qy, qz, qw)
        self.last_valid_time = 0.0
        self.cf = None                # Set when forwarding starts
        self._last_position = None
        self._repeat_count = 0

    def update(self, position, rotation):
        """Called from the NatNet thread. Returns the Crazyflie to forward
        the pose to, or None if the pose should not be forwarded."""
        position = tuple(position)
        rotation = tuple(rotation)
        with self.lock:
            # Real tracking always has some noise, so a pose that repeats
            # exactly for many frames means Motive is sending stale data.
            if position == self._last_position:
                self._repeat_count += 1
            else:
                self._repeat_count = 0
                self._last_position = position
            if self._repeat_count >= self.stale_frames:
                return None

            self.pose = position + rotation
            self.last_valid_time = time.time()
            return self.cf

    def tracking_ok(self):
        with self.lock:
            return (self.pose is not None and
                    time.time() - self.last_valid_time < self.tracking_timeout)

    def get_pose(self):
        with self.lock:
            return self.pose

    def get_position(self):
        pose = self.get_pose()
        return None if pose is None else pose[:3]

    def get_yaw_deg(self):
        pose = self.get_pose()
        return None if pose is None else quat_to_yaw_deg(*pose[3:])

    def safe_sleep(self, seconds):
        """Sleep, but raise TrackingLost if mocap stops seeing this drone."""
        end = time.time() + seconds
        while time.time() < end:
            if not self.tracking_ok():
                raise TrackingLost(self.uri)
            time.sleep(0.05)

    def wait_until_at(self, target, tolerance=0.10, timeout=3.0, label='target'):
        """Block until mocap shows the drone within tolerance (m) of target (x, y, z)."""
        deadline = time.time() + timeout
        while True:
            dist = math.dist(self.get_position(), target)
            if dist < tolerance:
                print(f'{self.uri} reached {label} (error {dist:.2f} m)')
                return
            if time.time() > deadline:
                raise RuntimeError(f'{self.uri} did not reach {label} (still {dist:.2f} m away)')
            self.safe_sleep(0.1)


# ----- The whole swarm

class Mocap:
    def __init__(self, drone_config, client_ip, server_ip,
                 tracking_timeout=0.3, stale_frames=10):
        """drone_config: {uri: Motive Streaming ID}. Its order is kept in .uris."""
        self.client_ip = client_ip
        self.server_ip = server_ip
        self.drones = {uri: DroneState(uri, rb_id, tracking_timeout, stale_frames)
                       for uri, rb_id in drone_config.items()}
        self._by_rb_id = {d.rb_id: d for d in self.drones.values()}
        self._client = None

    @property
    def uris(self):
        return list(self.drones)

    def drone(self, uri):
        return self.drones[uri]

    def drone_for(self, scf):
        return self.drones[scf.cf.link_uri]

    # --- NatNet

    def start(self):
        client = NatNetClient()
        client.set_client_address(self.client_ip)
        client.set_server_address(self.server_ip)
        client.set_use_multicast(False)
        client.set_print_level(0)
        client.rigid_body_listener = self._on_rigid_body
        if not client.run('d'):
            raise SystemExit("Failed to start NatNet client")
        self._client = client
        return self

    def shutdown(self):
        self.stop_forwarding()
        if self._client is not None:
            self._client.shutdown()
            self._client = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.shutdown()
        return False

    def _on_rigid_body(self, rb_id, position, rotation, tracking_valid=True):
        drone = self._by_rb_id.get(rb_id)
        if drone is None or not tracking_valid:
            return
        cf = drone.update(position, rotation)
        if cf is not None:
            x, y, z = position
            qx, qy, qz, qw = rotation
            cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

    # --- Forwarding (use with swarm.parallel_safe(mocap.start_forwarding))

    def start_forwarding(self, scf):
        drone = self.drone_for(scf)
        with drone.lock:
            drone.cf = scf.cf

    def stop_forwarding(self):
        # Doesn't go through the swarm, so it still works if the swarm is in a bad state
        for drone in self.drones.values():
            with drone.lock:
                drone.cf = None

    # --- Checks

    def wait_for_all(self, timeout=5.0, verbose=True):
        """Raise if any drone has no valid mocap data within timeout."""
        start = time.time()
        while True:
            missing = [d for d in self.drones.values() if not d.tracking_ok()]
            if not missing:
                break
            if time.time() - start > timeout:
                names = ', '.join(f'{d.uri} (ID {d.rb_id})' for d in missing)
                raise RuntimeError(f"No valid mocap data for: {names}. "
                                   "Check Motive streaming and Streaming IDs.")
            time.sleep(0.05)
        if verbose:
            for d in self.drones.values():
                x, y, z = d.get_position()
                print(f'Mocap sees {d.uri} at ({x:.2f}, {y:.2f}, {z:.2f})')


# ----- Drone-side setup for mocap

def reset_estimator(scf, orientation_std_dev=8.0e-3, timeout=10.0, strict=True):
    """Switch to Kalman, trust mocap orientation, reset, and wait until the
    filter's uncertainty settles. strict=False only warns on timeout.
    Requires mocap forwarding to already be running for this drone."""
    cf = scf.cf
    cf.param.set_value('stabilizer.estimator', '2')                  # Kalman
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    log = LogConfig(name='Kalman Variance', period_in_ms=500)
    log.add_variable('kalman.varPX', 'float')
    log.add_variable('kalman.varPY', 'float')
    log.add_variable('kalman.varPZ', 'float')
    history = [[1000.0] * 10 for _ in range(3)]
    start = time.time()

    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            for h, var in zip(history, ('kalman.varPX', 'kalman.varPY', 'kalman.varPZ')):
                h.append(data[var])
                h.pop(0)
            if all(max(h) - min(h) < 0.001 for h in history):
                return
            if time.time() - start > timeout:
                msg = f"Estimator did not converge on {cf.link_uri}. Is mocap data reaching the drone?"
                if strict:
                    raise RuntimeError(msg)
                print('WARNING: ' + msg)
                return


def quat_to_yaw_deg(qx, qy, qz, qw):
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                   1 - 2 * (qy * qy + qz * qz)))