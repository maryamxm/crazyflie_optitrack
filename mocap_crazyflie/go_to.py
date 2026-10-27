"""
Fly a Crazyflie to a target point in the Motive (OptiTrack) coordinate frame,
hover there, then land. Motive provides the drone's position and orientation,
which are streamed to the Crazyflie's onboard Kalman filter via mocap.py.

Usage:
    Set TARGET_X, TARGET_Y, TARGET_Z and HOVER_TIME in the settings, then run:
    python3 go_to.py

Requirements:
    - mocap.py next to this file

To Dos:
you need to put your device and the host device IP addresses
Drone's Motive ID
Input the drone radio info

"""

import math
import os
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from mocap import Mocap, TrackingLost, reset_estimator

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ----- SETTINGS

URI = uri_helper.uri_from_env(default= 'radio://<radio number>/<channel>/<data rate>/<address>')

# Target point in Motive coordinates (Z-up), in metres, and hover time
TARGET_X = 0.5
TARGET_Y = 0.5
TARGET_Z = 0.5
HOVER_TIME = 5.0              # s

DRONE_ID = 1                  # Streaming ID of the drone's rigid body in Motive
CLIENT_IP = "xxx.xxx.x.xxx"   # This laptop
SERVER_IP = "xxx.xxx.x.xx"    # The Motive PC

TAKEOFF_HEIGHT = 0.5          # m
SPEED = 0.3                   # m/s, used to compute the go_to duration
TRACKING_TIMEOUT = 0.3        # s without a valid pose -> treat tracking as lost
STALE_FRAMES = 10             # identical poses in a row -> treat data as frozen
ORIENTATION_STD_DEV = 8.0e-3  # How much the Kalman filter trusts mocap orientation

# Safe flight box in Motive coordinates (Z-up), in metres.
# Set these to stay well inside your tracked volume and away from walls/nets.
X_LIMITS = (-1.2, 1.6)
Y_LIMITS = (-2.1, 1.9)
Z_LIMITS = (0.2, 1.6)  # needs another checking and depends on where in the space it is


# ----- FLIGHT

def arm(cf):
    if hasattr(cf, 'supervisor'):
        cf.supervisor.send_arming_request(True)
    else:
        cf.platform.send_arming_request(True)
    time.sleep(1.0)


def land(cf, drone):
    hlc = cf.high_level_commander
    z = drone.get_position()[2]
    duration = max(z / SPEED, 2.0)
    hlc.land(0.0, duration)
    time.sleep(duration + 0.5)
    hlc.stop()


def fly(cf, drone, target, hover_time):
    hlc = cf.high_level_commander
    tx, ty, tz = target

    print(f"Taking off to {TAKEOFF_HEIGHT} m")
    hlc.takeoff(TAKEOFF_HEIGHT, 2.0)
    drone.safe_sleep(2.5)

    distance = math.dist(drone.get_position(), target)
    duration = max(distance / SPEED, 1.0)
    print(f"Flying to ({tx:.2f}, {ty:.2f}, {tz:.2f}), "
          f"{distance:.2f} m in {duration:.1f} s")
    hlc.go_to(tx, ty, tz, 0.0, duration, relative=False)
    drone.safe_sleep(duration + 0.5)

    print(f"Hovering for {hover_time} s")
    end = time.time() + hover_time
    while time.time() < end:
        x, y, z = drone.get_position()
        error = math.dist((x, y, z), target)
        print(f"  pos=({x:.2f}, {y:.2f}, {z:.2f})  error={error * 100:.1f} cm")
        drone.safe_sleep(1.0)

    print("Landing")
    land(cf, drone)


# ----- MAIN

def in_bounds(point):
    x, y, z = point
    return (X_LIMITS[0] <= x <= X_LIMITS[1] and
            Y_LIMITS[0] <= y <= Y_LIMITS[1] and
            Z_LIMITS[0] <= z <= Z_LIMITS[1])


def main():
    target = (TARGET_X, TARGET_Y, TARGET_Z)
    if not in_bounds(target):
        raise SystemExit(f"Target {target} is outside the safe flight box.")

    cflib.crtp.init_drivers()

    with Mocap({URI: DRONE_ID}, CLIENT_IP, SERVER_IP,
               tracking_timeout=TRACKING_TIMEOUT,
               stale_frames=STALE_FRAMES) as mocap:
        mocap.wait_for_all()             # Prints where mocap sees the drone
        drone = mocap.drone(URI)

        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=os.path.join(SCRIPT_DIR, 'cache'))) as scf:
            cf = scf.cf
            mocap.start_forwarding(scf)  # Start forwarding mocap poses to the drone
            try:
                reset_estimator(scf, orientation_std_dev=ORIENTATION_STD_DEV)

                arm(cf)
                print("Armed")

                try:
                    fly(cf, drone, target, HOVER_TIME)
                except TrackingLost:
                    print("Tracking lost! Landing.")
                    land(cf, drone)
                except KeyboardInterrupt:
                    print("Interrupted! Landing.")
                    land(cf, drone)
            finally:
                # Stop forwarding before the link closes; Mocap's own shutdown
                # only runs after SyncCrazyflie has already closed the link.
                mocap.stop_forwarding()


if __name__ == '__main__':
    main()