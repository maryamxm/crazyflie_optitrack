# `swarm_hover.py`: Swarm hover test

`swarm_hover.py` is the first swarm flight to run after single-drone tests. It connects every drone, runs the pre-flight checks, and makes all drones take off, hover and land, either in parallel or one at a time. It also has an optional square mode, where each drone flies a 1 m square relative to its takeoff point.

* **Placement:** drones are on the floor inside the flight area, at least ~0.3 m apart, with clear space above each.
* **Square mode only:** the area 1 m in +X and 1 m in +Y from every drone is clear and inside the tracked volume.

Run the script by `python3 swarm_hover.py` and press ctrl-c at any time to land all drones. Although, the drones will land eventually at the specified time. 

## Flight sequence

Mocap check → radio connection → start forwarding pose → LED check → reset estimators → compare estimate vs mocap → arm → check supervisor → fly → land → clean up

## Flight modes

**Parallel hover (default)**: All drones take off to TAKEOFF_HEIGHT together, hover for about 3 s, and land together.
```
swarm.parallel_safe(hover_sequence)
```

**Sequential hover**: One drone at a time takes off, hovers and lands, then the next one goes. Use this first with a new setup to check each drone separately. 
```
swarm.sequential(hover_sequence)
```

**Square**: All drones take off, then each flies a square of `box_size` in meters relative to its own position: +X, +Y, −X, −Y, at `flight_time` in seconds per side. Then all land.
```
swarm.parallel_safe(take_off)
swarm.parallel_safe(run_square_sequence)
swarm.parallel_safe(land)
```

> WARNING: Square mode has **no flight-volume check**. The square is relative, so each drone moves up to box_size m in +X and +Y from where it took off. Make sure every drone's square stays inside the tracked volume and doesn't overlap another drone's square. For flights in absolute coordinates with limit checks, use `swarm_square.py`.
