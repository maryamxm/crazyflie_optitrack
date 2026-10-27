#!/usr/bin/env python3
"""
Power control for multiple Crazyflies over Crazyradio.

Usage:
    python3 cf_power.py off          # soft off  (STM32 off, radio listening)
    python3 cf_power.py on           # wake from soft off
    python3 cf_power.py shutdown     # full platform power down (button needed to turn on)
    python3 cf_power.py reboot       # full reboot into firmware
    python3 cf_power.py cycle        # power-cycle (STM32 only)

    # Target specific drones by index (0-based) from the URIS list:
    python3 cf_power.py off -d 0 2
    # Or pass URIs directly:
    python3 cf_power.py on -u radio://0/80/2M/E7E7E7E7E1
"""

import argparse
import sys
import time

import cflib.crtp
from cflib.utils.power_switch import PowerSwitch

# ---- Edit this list to match your drones
URIS = [
    'radio://<radio number>/<channel>/<data rate>/<address>',
    'radio://<radio number>/<channel>/<data rate>/<address>',
    'radio://<radio number>/<channel>/<data rate>/<address>',
    'radio://<radio number>/<channel>/<data rate>/<address>',
]

ACTIONS = {
    'off': 'stm_power_down',
    'on': 'stm_power_up',
    'shutdown': 'platform_power_down',
    'reboot': 'reboot_to_fw',
    'cycle': 'stm_power_cycle',
}

RETRIES = 3
DELAY_BETWEEN = 0.2   # seconds between drones (avoids radio congestion)


def apply(uri: str, action: str) -> bool:
    method = ACTIONS[action]
    for attempt in range(1, RETRIES + 1):
        ps = None
        try:
            ps = PowerSwitch(uri)
            getattr(ps, method)()
            print(f'[OK]   {uri}  -> {action}')
            return True
        except Exception as e:
            print(f'[TRY {attempt}/{RETRIES}] {uri}  failed: {e}')
            time.sleep(0.3)
        finally:
            if ps is not None:
                try:
                    ps.close()
                except Exception:
                    pass
    print(f'[FAIL] {uri}  -> {action}')
    return False


def main():
    parser = argparse.ArgumentParser(description='Crazyflie swarm power control')
    parser.add_argument('action', choices=ACTIONS.keys())
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-d', '--drones', type=int, nargs='+',
                       help='indices into URIS (default: all)')
    group.add_argument('-u', '--uris', nargs='+', help='explicit URIs')
    args = parser.parse_args()

    if args.uris:
        targets = args.uris
    elif args.drones:
        try:
            targets = [URIS[i] for i in args.drones]
        except IndexError:
            sys.exit(f'Index out of range. URIS has {len(URIS)} entries.')
    else:
        targets = URIS

    if args.action == 'shutdown':
        print('Note: after shutdown, drones can only be turned on with the button.')

    cflib.crtp.init_drivers()

    failed = []
    for uri in targets:
        if not apply(uri, args.action):
            failed.append(uri)
        time.sleep(DELAY_BETWEEN)

    print(f'\nDone: {len(targets) - len(failed)}/{len(targets)} succeeded.')
    if failed:
        print('Failed:', *failed, sep='\n  ')
        sys.exit(1)


if __name__ == '__main__':
    main()