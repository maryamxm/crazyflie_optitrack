"""
In this code you can test the drones to hover sequentially, in parallel, or
fly in a square.

Sequence: check mocap -> connect -> start forwarding poses -> LED check ->
          reset estimators -> print estimated positions -> arm ->
          print supervisor -> take off -> land

Requirements:
    - mocap.py next to this file (all Motive/NatNet handling lives there)

To Dos:
you need to put your device and the host device IP addresses
Input the drones radio info and their corresponding Motive ID

"""

import os
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from mocap import Mocap, TrackingLost, reset_estimator

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ----- Settings

# URI -> Streaming ID of that drone's rigid body in Motive
DRONE_CONFIG = {
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
    'radio://<radio number>/<channel>/<data rate>/<address>': 1, #<ID>
}

CLIENT_IP = "xxx.xxx.x.xxx"   # This laptop
SERVER_IP = "xxx.xxx.x.xx"    # The Motive PC

TAKEOFF_HEIGHT = 0.5          # m

mocap = Mocap(DRONE_CONFIG, CLIENT_IP, SERVER_IP)
uris = mocap.uris


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


def take_off(scf):
    print(f'Takeoff command -> {scf.cf.link_uri}')
    commander = scf.cf.high_level_commander

    commander.takeoff(TAKEOFF_HEIGHT, 2.0)
    mocap.drone_for(scf).safe_sleep(3)


def land(scf):
    print(f'Land command -> {scf.cf.link_uri}')
    commander = scf.cf.high_level_commander

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()


def hover_sequence(scf):
    # Tracking loss is handled per drone, so one drone losing tracking
    # lands that drone without stopping the others.
    try:
        take_off(scf)
    except TrackingLost:
        print(f'Tracking lost on {scf.cf.link_uri}! Landing.')
    land(scf)


def run_square_sequence(scf):
    box_size = 1
    flight_time = 2

    commander = scf.cf.high_level_commander

    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)


# ----- Main

if __name__ == '__main__':
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

                # ----- hovering

                #swarm.sequential(hover_sequence)    # for a sequential flight
                swarm.parallel_safe(hover_sequence)  # for parallel flight

                # ----- square fly

                #swarm.parallel_safe(take_off)
                #swarm.parallel_safe(run_square_sequence)
                #swarm.parallel_safe(land)

                print('Flight is done')
            except BaseException:
                # Any error or Ctrl-C: land every drone, then show the error
                print('Aborting! Landing all drones.')
                swarm.parallel(land)    # parallel() ignores per-drone errors, so all get the command
                raise
            finally:
                mocap.stop_forwarding() # stop sending poses before the links close