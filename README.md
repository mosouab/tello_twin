# Tello digital twin: data-collection stack

ROS 2 Jazzy software to fly scripted flight plans on a simulated or real DJI/Ryze Tello and record
commands, Vicon ground truth and telemetry as a time-aligned dataset.
Master E3A project, Université d'Évry Paris-Saclay (IBISC), 2026-2027.

- Design: `docs/design/2026-11-21-tello-dataset-stack-design.md`
- Reference papers: `docs/papers/`
- Previous projects (not in git): `legacy/`

## Setup (host: CachyOS or any Linux with distrobox)

```bash
scripts/setup_jazzy.sh                                  # once: Ubuntu 24.04 distrobox "jazzy" with ROS 2 Jazzy + Gazebo Harmonic
scripts/in_jazzy.sh "colcon build --symlink-install"
```

`scripts/in_jazzy.sh "<command>"` runs a command inside the container with ROS, the venv
(`~/venvs/tello`) and the workspace sourced.

## Fly on the fake drone

Terminal 1, the session (drone node, Vicon adapter, emulated Vicon, and the safety console in an xterm):

```bash
scripts/in_jazzy.sh "ros2 launch tello_bringup session.launch.py backend:=fake session:=demo"
```

`console:=xterm` (the default) needs xterm in the container (`scripts/in_jazzy.sh "sudo apt-get install -y xterm"`); otherwise pass `console:=none`.

Terminal 2, one flight (dry run, then the recorder and the flight runner):

```bash
scripts/in_jazzy.sh "ros2 launch tello_bringup flight.launch.py plan:=src/tello_flight/plans/fake_short.yaml session:=demo"
```

- Each flight is written to `dataset/fake/raw/demo/demo_NN_<plan>/`: an MCAP bag (`bag/`) and `flight.yaml`.
- Safety console keys: **H** hold + abort, **R** release, **Space** land, **E** emergency, **T** takeoff.
- Volumes, limits and the Vicon topic are in `src/tello_bringup/config/lab.yaml`. Its dimensions are
  placeholders until the lab is measured.

## Tests

```bash
scripts/in_jazzy.sh "colcon test && colcon test-result --all"          # everything, including launch tests (~1.5 min)
scripts/in_jazzy.sh "cd src/tello_core && python3 -m pytest -q test"   # pure logic only (~1 s)
```

## Dry-run a flight plan

```bash
scripts/in_jazzy.sh "python3 -m tello_core.dry_run <plan.yaml> --soft-box XMIN XMAX YMIN YMAX ZMIN ZMAX --hard-box XMIN XMAX YMIN YMAX ZMIN ZMAX"
```

Exit codes: 0 = pass, 1 = fails the dry run, 2 = invalid plan. Pass the lab bring-up values as `--stick-max`
and `--cmd-delay` (defaults 0.4 and 0.7). `flight.launch.py` runs the same dry run with the values from
`lab.yaml`.

## Packages

| Package | Status | Contents |
|---|---|---|
| `tello_interfaces` | Plans 1-2 | Messages: `TelloTelemetry`, `TelloStatus`, `TelloEvent`, `FlightSegment` |
| `tello_core` | Plans 1-2 | Pure logic: sticks, SDK, safety, state machine, fake drone, signals, control, plans, executor, dry run, lab config, Vicon tracking, drone core, runner core |
| `tello_driver` | Plan 2 | `tello_node` with the `fake` backend; shared ROS helpers (`qos`, `ros_time`, `node_main`, `testing`) |
| `tello_mocap` | Plan 2 | `mocap_adapter` (validation, velocity) and `mocap_sim` (emulated Vicon) |
| `tello_flight` | Plan 2 | `flight_runner`, `safety_console`, flight plans |
| `tello_bringup` | Plan 2 | `lab.yaml`, `session.launch.py`, `flight.launch.py`, recording |
| `tello_dataset` | Plan 3 | Offline dataset builder |
| `tello_sim` | Plan 5 | Gazebo Harmonic world and drone model |
