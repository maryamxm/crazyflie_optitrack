"""
Swarm flight through a custom pattern using ABSOLUTE positions (Motive world frame).
Drones can be placed ANYWHERE in the flight area - each one is flown to its
start point automatically, one at a time.

Sequence: check mocap -> connect -> start forwarding poses -> LED check ->
          reset estimators -> print estimated positions -> arm -> print supervisor ->
          PHASE 1 (one drone at a time): take off to its own transit height,
                                         fly above its start point
          PHASE 2 (one drone at a time): descend to the flight height at its start point
          -> fly sequence in parallel -> land

Why transit heights: each drone gets its own height for the move, and drones
that go first get the higher ones. So:
    - a drone that takes off later always stops BELOW the ones already in the air
    - a drone flying to its start point never meets another drone at the same height
    - descending is vertical over a distinct start point, so nothing is in the way

This has been done so the drones can be placed anywhere in the flight area

Requirements:
    - mocap.py next to this file (all Motive/NatNet handling lives there)
    - Waypoints are in the Motive world frame (metres, origin = Motive ground plane origin)
    - Ceiling and mocap volume must cover the highest transit height
    - Drones on the floor at least ~0.3 m apart

To Dos:
you need to put your device and the host device IP addresses
Input the drones radio info and their corresponding Motive ID

"""

import math
import os
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from mocap import Mocap, reset_estimator

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ----- Settings

# URI -> Streaming ID of that drone's rigid body in Motive
# The order here is also the order the drones move to their start points.
DRONE_CONFIG = {
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
}

CLIENT_IP = "xxx.xxx.x.xxx"   # This laptop
SERVER_IP = "xxx.xxx.x.xx"    # The Motive PC

FLIGHT_HEIGHT = 0.5          # m (absolute), height of the pattern

# Transit heights: first drone gets the highest, last drone the lowest.

TRANSIT_BASE = 0.65           # m, lowest transit height (must be above FLIGHT_HEIGHT by at least a TRANSIT_SPACING)
TRANSIT_SPACING = 0.15        # m, vertical gap between transit heights
VERTICAL_SPEED = 0.4          # m/s, for takeoff and descent durations
HORIZONTAL_SPEED = 0.4        # m/s, for the move to above the start point
MIN_MOVE_TIME = 1.5           # s, shortest allowed move

ARRIVAL_TOLERANCE = 0.10      # m, 3D distance (mocap) that counts as "in position"
ARRIVAL_TIMEOUT = 3.0         # s, extra time allowed after a move to reach the tolerance

# Flight volume limits (Motive world frame). Any waypoint outside -> script refuses to run.
X_LIMITS = (-1.2, 1.6)
Y_LIMITS = (-2.1, 1.9)
Z_LIMITS = (0.2, 1.6)  # needs another checking and depends on where in the space it is

mocap = Mocap(DRONE_CONFIG, CLIENT_IP, SERVER_IP)
uris = mocap.uris


def transit_height(uri):
    """First drone in uris -> highest, last -> TRANSIT_BASE."""
    return TRANSIT_BASE + TRANSIT_SPACING * (len(uris) - 1 - uris.index(uri))


# ----- Absolute pattern

h = FLIGHT_HEIGHT     # absolute flight height

P0 = (-0.5, +0.5)
P1 = (+0.5, +0.5)
P2 = (+0.5, -0.5)
P3 = (-0.5, -0.5)

# Each drone starts at its own corner and moves one corner clockwise per step,
# so all four stay on the square's edges at the same time and never cross.
# First waypoint = the drone's start point (reached sequentially before the pattern).
#              x       y     z   time
sequence0 = [
    (*P0, h, 3.0),
    (*P1, h, 3.0),
    (*P2, h, 3.0),
    (*P3, h, 3.0),
    (*P0, h, 3.0),
]

sequence1 = [
    (*P1, h, 3.0),
    (*P2, h, 3.0),
    (*P3, h, 3.0),
    (*P0, h, 3.0),
    (*P1, h, 3.0),
]

sequence2 = [
    (*P2, h, 3.0),
    (*P3, h, 3.0),
    (*P0, h, 3.0),
    (*P1, h, 3.0),
    (*P2, h, 3.0),
]

sequence3 = [
    (*P3, h, 3.0),
    (*P0, h, 3.0),
    (*P1, h, 3.0),
    (*P2, h, 3.0),
    (*P3, h, 3.0),
]

SEQUENCES = {
    uris[0]: sequence0,
    uris[1]: sequence1,
    uris[2]: sequence2,
    uris[3]: sequence3,
}

# The first waypoint of each sequence is the start point, reached in phases 1-2.
# The parallel part only flies the remaining waypoints.
seq_args = {uri: [seq[1:]] for uri, seq in SEQUENCES.items()}


def in_volume(x, y, z):
    return (X_LIMITS[0] <= x <= X_LIMITS[1] and
            Y_LIMITS[0] <= y <= Y_LIMITS[1] and
            Z_LIMITS[0] <= z <= Z_LIMITS[1])


def validate_sequences():
    """Refuse to fly if any waypoint or transit height is outside the flight volume."""
    if TRANSIT_BASE - FLIGHT_HEIGHT < TRANSIT_SPACING:
        raise RuntimeError("TRANSIT_BASE must be at least TRANSIT_SPACING above FLIGHT_HEIGHT")
    for uri, seq in SEQUENCES.items():
        for x, y, z, t in seq:
            if not in_volume(x, y, z):
                raise RuntimeError(f"Waypoint ({x}, {y}, {z}) for {uri} is outside the flight volume")
            if t <= 0:
                raise RuntimeError(f"Non-positive duration in sequence for {uri}")
        sx, sy = seq[0][:2]
        if not in_volume(sx, sy, transit_height(uri)):
            raise RuntimeError(f"Transit height {transit_height(uri):.2f} m for {uri} "
                               "is outside the flight volume")
    starts = [seq[0][:2] for seq in SEQUENCES.values()]
    if len(set(starts)) != len(starts):
        raise RuntimeError("Two drones share the same start point")


def move_time(distance, speed):
    return max(MIN_MOVE_TIME, distance / speed)


# ----- Drone functions

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)


def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)


def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)


# Compare each drone's estimate with mocap
def print_estimated_position(scf):
    log = LogConfig(name='pos', period_in_ms=100)
    log.add_variable('stateEstimate.x', 'float')
    log.add_variable('stateEstimate.y', 'float')
    log.add_variable('stateEstimate.z', 'float')
    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            ex, ey, ez = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
            break
    mx, my, mz = mocap.drone_for(scf).get_position()
    print(f'{scf.cf.link_uri}  Estimate: ({ex:.2f}, {ey:.2f}, {ez:.2f})   '
          f'Mocap: ({mx:.2f}, {my:.2f}, {mz:.2f})')


def print_supervisor(scf):
    log = LogConfig(name='sup', period_in_ms=100)
    log.add_variable('supervisor.info', 'uint16_t')
    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            info = data['supervisor.info']
            break
    names = ['can_be_armed', 'is_armed', 'auto_arm', 'can_fly',
             'is_flying', 'is_tumbled', 'is_locked', 'is_crashed']
    active = [n for i, n in enumerate(names) if info & (1 << i)]
    print(f'{scf.cf.link_uri}: {active}')


def arm(scf):
    scf.cf.supervisor.send_arming_request(True)
    time.sleep(1.0)


def land(scf):
    print(f'Land command -> {scf.cf.link_uri}')
    commander = scf.cf.high_level_commander

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()


def go_to_transit(scf: SyncCrazyflie):
    """PHASE 1. Take off straight up to this drone's transit height, then fly
    horizontally to above its start point. Called with swarm.sequential()."""
    cf = scf.cf
    drone = mocap.drone_for(scf)
    commander = cf.high_level_commander
    zt = transit_height(cf.link_uri)
    sx, sy = SEQUENCES[cf.link_uri][0][:2]
    x0, y0, _ = drone.get_position()

    print(f'Takeoff {cf.link_uri} to transit height {zt:.2f} m')
    t = move_time(zt, VERTICAL_SPEED)
    commander.takeoff(zt, t)                       # absolute height
    drone.safe_sleep(t)
    drone.wait_until_at((x0, y0, zt), ARRIVAL_TOLERANCE, ARRIVAL_TIMEOUT, 'transit height')

    print(f'Moving {cf.link_uri} above start point ({sx:.2f}, {sy:.2f})')
    t = move_time(math.hypot(sx - x0, sy - y0), HORIZONTAL_SPEED)
    commander.go_to(sx, sy, zt, 0, t, relative=False)
    drone.safe_sleep(t)
    drone.wait_until_at((sx, sy, zt), ARRIVAL_TOLERANCE, ARRIVAL_TIMEOUT, 'above start point')


def descend_to_start(scf: SyncCrazyflie):
    """PHASE 2. Descend vertically to the start point at flight height.
    Called with swarm.sequential()."""
    cf = scf.cf
    drone = mocap.drone_for(scf)
    x, y, z = SEQUENCES[cf.link_uri][0][:3]
    zt = transit_height(cf.link_uri)

    print(f'Descending {cf.link_uri} to start point ({x:.2f}, {y:.2f}, {z:.2f})')
    t = move_time(zt - z, VERTICAL_SPEED)
    cf.high_level_commander.go_to(x, y, z, 0, t, relative=False)
    drone.safe_sleep(t)
    drone.wait_until_at((x, y, z), ARRIVAL_TOLERANCE, ARRIVAL_TIMEOUT, 'start point')


def run_sequence(scf: SyncCrazyflie, sequence):
    cf = scf.cf
    commander = cf.high_level_commander
    drone = mocap.drone_for(scf)

    for x, y, z, duration in sequence:
        print('Going to absolute position {} with cf {}'.format((x, y, z), cf.link_uri))
        commander.go_to(x, y, z, 0, duration, relative=False)
        drone.safe_sleep(duration)    # aborts (-> all land) if this drone loses tracking


# ----- Main

if __name__ == '__main__':
    validate_sequences()
    cflib.crtp.init_drivers()

    with mocap:                                         # starts NatNet, shuts it down on exit
        mocap.wait_for_all()
        print('Mocap check passed')

        factory = CachedCfFactory(rw_cache=os.path.join(SCRIPT_DIR, 'cache'))
        with Swarm(uris, factory=factory) as swarm:
            print('Connected to Crazyflies')
            try:
                swarm.parallel_safe(mocap.start_forwarding)   # start sending mocap poses
                swarm.parallel_safe(light_check)
                print('LED check passed')
                swarm.parallel_safe(reset_estimator)
                print('Estimators reset')
                swarm.sequential(print_estimated_position)
                swarm.parallel_safe(arm)
                swarm.sequential(print_supervisor)

                swarm.sequential(go_to_transit)         # phase 1, one drone at a time
                print('All drones above their start points')
                swarm.sequential(descend_to_start)      # phase 2, one drone at a time
                print('All drones in position, starting sequence')

                swarm.parallel_safe(run_sequence, args_dict=seq_args)
                swarm.parallel_safe(land)

                print('Flight is done')
            except BaseException:
                # Any error or Ctrl-C: land every drone, then show the error
                print('Aborting! Landing all drones.')
                swarm.parallel(land)    # parallel() ignores per-drone errors, so all get the command
                raise
            finally:
                mocap.stop_forwarding() # stop sending poses before the links close