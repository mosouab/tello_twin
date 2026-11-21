# Tello digital twin: data-collection stack

ROS 2 Jazzy software to fly scripted flight plans on a simulated or real DJI/Ryze Tello and record
commands, Vicon ground truth and telemetry as a time-aligned dataset.
Master E3A project, Université d'Évry Paris-Saclay (IBISC), 2026-2027.

- Design: `docs/design/2026-11-21-tello-dataset-stack-design.md`
- Reference papers: `docs/papers/`
- Previous projects (not in git): `legacy/`

## Setup (host: CachyOS or any Linux with distrobox)

```bash
scripts/setup_jazzy.sh          # once: Ubuntu 24.04 distrobox "jazzy" with ROS 2 Jazzy + Gazebo Harmonic
scripts/in_jazzy.sh "colcon build"
```

`scripts/in_jazzy.sh "<command>"` runs a command inside the container with ROS, the venv
(`~/venvs/tello`) and the workspace sourced.

## Tests

```bash
scripts/in_jazzy.sh "cd src/tello_core && python3 -m pytest -q test"
```

## Dry-run a flight plan

```bash
scripts/in_jazzy.sh "python3 -m tello_core.dry_run <plan.yaml> --soft-box XMIN XMAX YMIN YMAX ZMIN ZMAX --hard-box XMIN XMAX YMIN YMAX ZMIN ZMAX"
```

Exit codes: 0 = pass, 1 = fails the dry run, 2 = invalid plan. Pass the lab bring-up values as `--stick-max` and `--cmd-delay` (defaults 0.4 and 0.7); the dry run fails if the plan's `limits.stick_max` exceeds them. `python3 -m tello_core.dry_run` from the repo root runs the copy installed by colcon, so rebuild after editing `tello_core` (or build with `--symlink-install`).

## Packages

| Package | Status | Contents |
|---|---|---|
| `tello_interfaces` | Plan 1 | Messages: `TelloTelemetry`, `TelloStatus`, `TelloEvent`, `FlightSegment` |
| `tello_core` | Plan 1 | Pure logic: sticks, SDK parsing, safety, state machine, fake drone, signals, control, plans, executor, dry run |
| `tello_driver`, `tello_mocap`, `tello_flight`, `tello_bringup` | Plan 2 | ROS nodes, launch files and recording |
| `tello_dataset` | Plan 3 | Offline dataset builder |
| `tello_sim` | Plan 5 | Gazebo Harmonic world and drone model |
