"""
Status monitor for the Crazyflie swarm. Good for debugging. 

Sections you can show (default: all of them):
    battery     - voltage, level (%), power state
    mocap       - position + yaw straight from Motive
    estimate    - onboard Kalman position + yaw (fed by mocap)
    error       - distance between estimate and mocap
    kalman      - Kalman position uncertainty (std dev per axis)
    supervisor  - flags (can_be_armed, is_armed, is_tumbled, ...)

Shortcuts:
    pose        = mocap + estimate + error
    all         = everything (default)

Usage:
    python3 swarm_status.py                          # everything, one snapshot
    python3 swarm_status.py --show battery           # battery only (no Motive needed)
    python3 swarm_status.py --show pose supervisor   # combine sections
    python3 swarm_status.py --show battery --watch   # live table, Ctrl-C to stop
    python3 swarm_status.py --watch --rate 0.5

Needs Motive --> mocap, estimate, error, kalman
Does not need Motive --> Battery, setpoint, supervisor

Requirements:
    - mocap.py next to this file (all Motive/NatNet handling lives there)

To Dos (settings section):
you need to put your device and the host device IP addresses
Input the drones radio info and their corresponding Motive ID 

"""

import argparse
import contextlib
import functools
import math
import os
import threading
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

from mocap import Mocap, reset_estimator

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

LOG_PERIOD_MS = 100           # onboard log rate
LOW_BATTERY_V = 3.5           # resting voltage below this -> flagged LOW

PM_STATES = {0: 'battery', 1: 'charging', 2: 'charged', 3: 'LOW POWER', 4: 'shutdown'}
SUPERVISOR_FLAGS = ['can_be_armed', 'is_armed', 'auto_arm', 'can_fly',
                    'is_flying', 'is_tumbled', 'is_locked', 'is_crashed']

# ----- Sections

SECTIONS = ['battery', 'mocap', 'estimate', 'error', 'kalman', 'setpoint', 'supervisor']
SHORTCUTS = {
    'pose': ['mocap', 'estimate', 'error'],
    'all': SECTIONS,
}

ESTIMATE_VARS = [('stateEstimate.x', 'float'), ('stateEstimate.y', 'float'),
                 ('stateEstimate.z', 'float'), ('stateEstimate.yaw', 'float')]

# Onboard log variables each section needs
SECTION_VARS = {
    'battery': [('pm.vbat', 'float'), ('pm.batteryLevel', 'uint8_t'), ('pm.state', 'int8_t')],
    'mocap': [],
    'estimate': ESTIMATE_VARS,
    'error': ESTIMATE_VARS,
    'kalman': [('kalman.varPX', 'float'), ('kalman.varPY', 'float'), ('kalman.varPZ', 'float')],
    'setpoint': [('ctrltarget.x', 'float'), ('ctrltarget.y', 'float'), ('ctrltarget.z', 'float')],
    'supervisor': [('supervisor.info', 'uint16_t')],
}

# Sections that are meaningless without Motive streaming to the drones
NEEDS_MOCAP = {'mocap', 'estimate', 'error', 'kalman'}

TYPE_SIZE = {'float': 4, 'uint8_t': 1, 'int8_t': 1, 'uint16_t': 2}
MAX_BLOCK_BYTES = 26          # cflib limit per log block

mocap = Mocap(DRONE_CONFIG, CLIENT_IP, SERVER_IP)

# Latest onboard log values per drone, written by cflib log threads
onboard = {uri: {} for uri in mocap.uris}
onboard_lock = threading.Lock()


def expand_sections(requested):
    """Turn e.g. ['pose', 'battery'] into ordered, de-duplicated sections."""
    wanted = set()
    for name in requested:
        wanted.update(SHORTCUTS.get(name, [name]))
    return [s for s in SECTIONS if s in wanted]


def plan_log_blocks(sections):
    """Pack the needed variables into blocks of at most 26 bytes."""
    variables = []
    for s in sections:
        for var in SECTION_VARS[s]:
            if var not in variables:
                variables.append(var)

    blocks, current, size = [], [], 0
    for name, vtype in variables:
        n = TYPE_SIZE[vtype]
        if size + n > MAX_BLOCK_BYTES:
            blocks.append(current)
            current, size = [], 0
        current.append((name, vtype))
        size += n
    if current:
        blocks.append(current)
    return blocks


# ----- Onboard logging

def start_status_logs(scf, blocks):
    uri = scf.cf.link_uri

    def on_data(_timestamp, data, _logconf):
        with onboard_lock:
            onboard[uri].update(data)

    for i, block in enumerate(blocks):
        log = LogConfig(name=f'status{i}', period_in_ms=LOG_PERIOD_MS)
        for name, vtype in block:
            log.add_variable(name, vtype)
        scf.cf.log.add_config(log)
        log.data_received_cb.add_callback(on_data)
        log.start()


def reset_estimator_lenient(scf):
    # Status check should still show everything if one drone's filter is slow
    reset_estimator(scf, strict=False)


# ----- Display

def fmt_xyz(x, y, z):
    return f'({x:+.3f}, {y:+.3f}, {z:+.3f})'


def report(sections):
    lines = [time.strftime('%H:%M:%S'), '']
    for uri, d in mocap.drones.items():
        with onboard_lock:
            ob = dict(onboard[uri])
        pos = d.get_position() if 'mocap' in sections or 'error' in sections else None
        est = None
        if 'stateEstimate.x' in ob:
            est = (ob['stateEstimate.x'], ob['stateEstimate.y'], ob['stateEstimate.z'])

        lines.append(f'=== {uri}  (Motive ID {d.rb_id})')

        if 'battery' in sections:
            if 'pm.vbat' in ob:
                v = ob['pm.vbat']
                state = PM_STATES.get(ob['pm.state'], str(ob['pm.state']))
                flag = '   <-- LOW' if v < LOW_BATTERY_V else ''
                lines.append(f'  Battery    : {v:.2f} V  {ob["pm.batteryLevel"]:3d} %  [{state}]{flag}')
            else:
                lines.append('  Battery    : no data yet')

        if 'mocap' in sections:
            if pos is not None:
                status = '' if d.tracking_ok() else '   <-- TRACKING LOST (last known)'
                lines.append(f'  Mocap      : {fmt_xyz(*pos)}  yaw {d.get_yaw_deg():+7.1f} deg{status}')
            else:
                lines.append('  Mocap      : NOT SEEN')

        if 'estimate' in sections:
            if est is not None:
                lines.append(f'  Estimate   : {fmt_xyz(*est)}  yaw {ob["stateEstimate.yaw"]:+7.1f} deg')
            else:
                lines.append('  Estimate   : no data yet')

        if 'error' in sections:
            if est is not None and pos is not None:
                lines.append(f'  Est. error : {math.dist(est, pos):.3f} m')
            else:
                lines.append('  Est. error : n/a (need both mocap and estimate)')

        if 'kalman' in sections:
            if 'kalman.varPX' in ob:
                sx, sy, sz = (math.sqrt(max(ob[k], 0.0))
                              for k in ('kalman.varPX', 'kalman.varPY', 'kalman.varPZ'))
                lines.append(f'  Kalman std : ({sx:.4f}, {sy:.4f}, {sz:.4f}) m')
            else:
                lines.append('  Kalman std : no data yet')

        if 'setpoint' in sections:
            if 'ctrltarget.x' in ob:
                sp = (ob['ctrltarget.x'], ob['ctrltarget.y'], ob['ctrltarget.z'])
                lines.append(f'  Setpoint   : {fmt_xyz(*sp)}')
            else:
                lines.append('  Setpoint   : no data yet')

        if 'supervisor' in sections:
            if 'supervisor.info' in ob:
                info = ob['supervisor.info']
                active = [n for i, n in enumerate(SUPERVISOR_FLAGS) if info & (1 << i)]
                lines.append(f'  Supervisor : {active}')
            else:
                lines.append('  Supervisor : no data yet')

        lines.append('')
    return '\n'.join(lines)


# ----- Main

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Crazyflie swarm status monitor')
    parser.add_argument('--show', '-s', nargs='+', default=['all'],
                        choices=SECTIONS + list(SHORTCUTS),
                        help='sections to display (default: all)')
    parser.add_argument('--watch', action='store_true', help='keep refreshing until Ctrl-C')
    parser.add_argument('--rate', type=float, default=1.0, help='refresh period in seconds (watch mode)')
    args = parser.parse_args()

    sections = expand_sections(args.show)
    blocks = plan_log_blocks(sections)
    use_mocap = bool(NEEDS_MOCAP & set(sections))

    cflib.crtp.init_drivers()

    with (mocap if use_mocap else contextlib.nullcontext()):
        factory = CachedCfFactory(rw_cache=os.path.join(SCRIPT_DIR, 'cache'))
        with Swarm(mocap.uris, factory=factory) as swarm:
            print('Connected to Crazyflies')
            print('Showing:', ', '.join(sections))
            try:
                if use_mocap:
                    swarm.parallel_safe(mocap.start_forwarding)   # estimate is only meaningful with mocap
                    swarm.parallel_safe(reset_estimator_lenient)
                    print('Estimators reset')
                if blocks:
                    swarm.parallel_safe(functools.partial(start_status_logs, blocks=blocks))
                    time.sleep(0.5)                               # let the first log packets arrive

                if args.watch:
                    while True:
                        print('\033[2J\033[H' + report(sections), flush=True)   # clear screen + redraw
                        time.sleep(args.rate)
                else:
                    print(report(sections))
            except KeyboardInterrupt:
                print('\nStopped')
            finally:
                if use_mocap:
                    mocap.stop_forwarding()