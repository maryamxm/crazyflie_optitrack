"""
Receives mocap pose from Motive for the given drone ID as x, y, z and a quaternion

Usage:
    Set the IP addresses and the drones ID, then run:
    python3 drone_pose.py

"""

import time
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'PythonClient'))
from NatNetClient import NatNetClient

DRONE_ID = 2

def on_rigid_body(rb_id, position, rotation):
    if rb_id == DRONE_ID:
        x, y, z = position
        qx, qy, qz, qw = rotation
        print(f"x={x:.2f} y={y:.2f} z={z:.2f} | "
              f"q=({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})")

client = NatNetClient()
client.set_client_address("xxx.xxx.x.xxx")
client.set_server_address("xxx.xxx.x.xx")
client.set_use_multicast(False)          # Unicast, matching Motive
client.set_print_level(0)                # silence the sample's own output
client.rigid_body_listener = on_rigid_body

if not client.run('d'):
    print("Failed to start client")
    raise SystemExit

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    client.shutdown() 
    