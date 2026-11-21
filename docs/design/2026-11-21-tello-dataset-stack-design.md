# Tello twin: data-collection stack (ROS 2 Jazzy). Design

- Date: 2026-11-21
- Status: approved in conversation, waiting for review of this written spec
- Scope: first milestone of the 2026-2027 Tello digital-twin project (Master E3A, Univ. Évry, IBISC): collecting the flight dataset

## 1. Goal

Build ROS 2 Jazzy software that flies scripted flight plans on a simulated or a real DJI/Ryze Tello. It records commands, motion-capture ground truth (Vicon) and drone telemetry, and turns each flight into a single file where all signals share one clock and the alignment error is measured.

The dataset is the input to the later work (model identification, learned and hybrid models, online twin). The design builds on the lessons from the 2025-2026 projects and the supervisor's review (`legacy/`), and on the two reference documents in `docs/papers/`:

- Švrčina, *Identification and Flight Control of Robotic Helicopter*, CTU 2024. A standard Tello flown with Vicon. Findings: about 0.6-0.7 s of delay between a command and the motion, commands sometimes lost over Wi-Fi, a dead zone near zero stick, and noisy Vicon heading on such a small drone.
- Ullah & Baca, *NanoBench*, arXiv 2603.09908 (2026). A Crazyflie flown with Vicon. Method used here: stamp everything on the host when it arrives, find the remaining offset by cross-correlating onboard and Vicon signals, and publish one file per flight on one time axis.

### 1.1 Success criteria

1. **Simulation end-to-end.** `session.launch.py backend:=sim` plus `flight.launch.py plan:=sysid_basic.yaml` produce a bag and `flight.yaml`. The builder turns them into a Parquet file and `quality.json`. The Vicon clock offset and drift injected in simulation are recovered within 2 ms, and the injected telemetry delay within 30 ms.
2. **Real protocol.** With `backend:=sdk` pointed at `fake_tello_udp`, the protocol tests pass (command sequence, `rc` format and rate, lost replies, missing telemetry).
3. **Safety.** The safety tests pass on the `fake` and `sim` backends: land within 1.5 s after losing Vicon, hover within 0.3 s after commands stop, override on the predicted flight-volume boundary, battery rules, and the keyboard console keys H, Space and E.
4. **Hardware bring-up.** The bench step and the hand-carry step (section 11) are done, and their `clock_check` report is saved.
5. **First real dataset.** At least one real flight is processed into Parquet with a `quality.json` that passes the thresholds in section 9.4.
6. **Low-level protocol.** The bench-test report gives a go or no-go decision (section 12).

### 1.2 In scope / out of scope

In scope:
- the drone abstraction (`sim`, `sdk`, `fake` backends)
- Vicon input from a separate computer
- time alignment
- flight plans and the safety layer
- the keyboard safety console
- recording
- the offline dataset builder
- the Harmonic simulation
- tests and the hardware bring-up procedure
- the low-level protocol bench test

Out of scope for v1 (section 14 lists them as future work): video, gamepad control, `track` segments, a `lowlevel` backend, multiple drones, the online twin, re-tuning the simulation from identified parameters.

## 2. Environment and network

### 2.1 Software

- **Host:** CachyOS, which ROS doesn't support. All ROS work runs in a **distrobox container with Ubuntu 24.04**, called `jazzy`. It shares the host network and `/dev/dri` (Intel Iris Xe for Gazebo rendering).
- **System packages in the container:**
  - `ros-jazzy-desktop`
  - `ros-jazzy-ros-gz` (Gazebo Harmonic)
  - `ros-jazzy-rosbag2-storage-mcap`
  - `ros-jazzy-vrpn-mocap` (for the fallback Vicon source, section 6.1; built from source if the apt package is missing)
  - `python3-venv`
  - `python3-pytest`
- **Python virtual environment:** `~/venvs/tello`, created with `--system-site-packages` so it sees `rclpy`. It holds `numpy`, `scipy`, `pandas`, `pyarrow`, `pyyaml`, `mcap` and `mcap-ros2-support`.
- **ROS middleware:** Fast DDS, the Jazzy default.

### 2.2 Network layout

| Interface | Connects to | Used for |
|---|---|---|
| Tello USB Wi-Fi dongle | The drone's network `TELLO-XXXXXX` (drone at 192.168.10.1) | SDK UDP: commands on 8889, state on 8890 |
| USB-C Ethernet hub | The lab network (Vicon computer) | ROS 2 / DDS traffic only |
| Built-in Wi-Fi | Internet (optional) | Not used by the stack |

- **Keep DDS off the Tello network:**
  - `ROS_DOMAIN_ID` is set to the lab's value.
  - `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.
  - A Fast DDS XML profile (`tello_bringup/config/fastdds_lab.xml`) whitelists the Ethernet interface.
  - `ROS_STATIC_PEERS=<Vicon computer IP>` is added if discovery fails.
- **SDK sockets bind to the IP of the dongle interface**, so the right route is used whatever the default route is.
- **The lab network must not use 192.168.10.0/24.** This is checked during bring-up.
- **The host firewall must allow** incoming UDP on 8890 and DDS on the Ethernet interface. This is checked during bring-up.

## 3. Architecture

```
 flight_runner ── /tello/cmd ──►┌────────────── tello_node ──────────────┐── sdk  ── UDP ──► Tello
 (flight plan YAML)             │ source priority · stick limits · SDK   │── sim  ── ros_gz ──► Gazebo Harmonic
 safety_console ── services ───►│ rounding · 50 Hz send · state machine  │── fake ── in-process model (tests)
                                │ watchdog · flight volume · failsafes   │
                                └─────────────────────────────────────────┘
 Vicon computer ── Ethernet ── <raw Vicon topic> ──► mocap_adapter ──► /tello/mocap/pose, /tello/mocap/odom
 (fallback) Vicon Tracker ── VRPN :3883 ──► vrpn_mocap on the laptop ──► /vrpn_mocap/<body>/pose ──► mocap_adapter
 sim/fake ground truth /sim/truth/odom ──► mocap_sim ──► <raw Vicon topic> (same name and type as the lab)
 clock_check ◄── <raw Vicon topic>
 rosbag2 (MCAP), one per flight ──► tello_dataset builder (offline, ROS-free) ──► Parquet + quality.json
```

### 3.1 Packages (colcon workspace: `src/` at the repository root)

| Package | Type | Contents | ROS dependency |
|---|---|---|---|
| `tello_interfaces` | ament_cmake | `TelloTelemetry`, `TelloStatus`, `TelloEvent`, `FlightSegment` messages | yes |
| `tello_core` | ament_python | Pure logic: stick conversion and rounding, safety rules, flight state machine, sync algorithms, flight-plan schema and signal generators, `fake` dynamics | **none** (installable with pip) |
| `tello_driver` | ament_python | `tello_node`, backends `sdk` / `sim` / `fake`, test server `fake_tello_udp` | yes |
| `tello_mocap` | ament_python | `mocap_adapter`, `mocap_sim`, `clock_check` | yes |
| `tello_flight` | ament_python | `flight_runner`, `safety_console`, example flight plans | yes |
| `tello_sim` | ament_cmake (data only) | Harmonic world, Tello SDF, ros_gz_bridge config | yes |
| `tello_bringup` | ament_python | Launch files `session`, `flight`, `sim`; configs for the lab, Vicon, flight volume and DDS | yes |
| `tello_dataset` | ament_python | Offline builder, readers, quality report | **none** (installable with pip) |

`tello_core` and `tello_dataset` never import `rclpy`. That way the logic can be tested with plain pytest, and the dataset can be processed on any machine. The existing empty folder `src/ros_tello/` isn't used.

## 4. Interfaces

### 4.1 Conventions

- **Stick commands** are `geometry_msgs/TwistStamped` normalised to [-1, 1], with ROS directions (REP 103):

  | Field | Meaning |
  |---|---|
  | `linear.x` | forward |
  | `linear.y` | left |
  | `linear.z` | up |
  | `angular.z` | yaw, counter-clockwise (left) |

- **Conversion to the SDK** (`rc a b c d`, integers from -100 to 100):

  | SDK value | From stick |
  |---|---|
  | `a` | `-round(100*y)`, positive = right |
  | `b` | `round(100*x)` |
  | `c` | `round(100*z)` |
  | `d` | `-round(100*yaw)`, positive = clockwise |

  The signs are verified in bring-up step 3.
- **Arena frame `arena`:** the Vicon world frame, z up, origin on the floor. Verified at the first lab visit.
- **Body frame:** x forward. The yaw offset between the Vicon rigid body and the drone's real heading is estimated offline (section 9.2). Online, it is a parameter.

### 4.2 Topics and services (namespace `/tello`)

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/tello/cmd` | `geometry_msgs/TwistStamped` | in | Commands from `flight_runner` |
| `/tello/cmd_sent` | `geometry_msgs/TwistStamped` | out | **Exactly what the backend sent**, after source priority, limits and SDK rounding (values = integer/100). Stamped on the laptop clock just before sending. This is the dataset's command signal. |
| `/tello/telemetry` | `tello_interfaces/TelloTelemetry` | out | Parsed state, SI units, header = arrival time |
| `/tello/telemetry_raw` | `std_msgs/String` | out | The raw SDK state string, for re-parsing later |
| `/tello/status` | `tello_interfaces/TelloStatus` | out | 10 Hz, plus on every change |
| `/tello/events` | `tello_interfaces/TelloEvent` | out | Safety and flight events, transient-local QoS |
| `/tello/flight/segment` | `tello_interfaces/FlightSegment` | out (runner) | Published when each segment starts, transient-local QoS |
| `/tello/mocap/pose` | `geometry_msgs/PoseStamped` | out (adapter) | Validated pose. The header stamp is the **source stamp, never rewritten** |
| `/tello/mocap/odom` | `nav_msgs/Odometry` | out (adapter) | Pose plus filtered velocity, for online use (flight volume, `goto`, `arena`) |
| `/tello/takeoff`, `/tello/land`, `/tello/emergency` | `std_srvs/Trigger` | services | `emergency` stops the motors immediately |
| `/tello/hold` | `std_srvs/SetBool` | service | true: ignore `/tello/cmd` and send zero sticks. false: accept commands again |
| `/flight_runner/abort` | `std_srvs/Trigger` | service | Stops the plan; the runner then sends zero sticks and requests landing |
| `/sim/truth/odom` | `nav_msgs/Odometry` | sim/fake only | Ground truth, read by `mocap_sim` |
| `/sim/backend/config`, `/sim/mocap_sim/config` | `std_msgs/String` (YAML) | sim/fake only | The injected delays, offsets and noise, for checking that the builder recovers them |

### 4.3 Messages

`TelloTelemetry.msg`:

```
std_msgs/Header header      # stamp = arrival on the laptop (sim: sim time)
float32 roll                # rad   (SDK roll, deg)
float32 pitch               # rad   (SDK pitch, deg)
float32 yaw                 # rad   (SDK yaw, deg; sign converted to CCW positive)
float32 vel_x               # m/s   (SDK vgx dm/s; axis mapping documented in tello_core, verified by hand-carry test)
float32 vel_y               # m/s   (SDK vgy)
float32 vel_z               # m/s   (SDK vgz)
float32 acc_x               # m/s^2 (SDK agx, 0.001 g)
float32 acc_y               # m/s^2 (SDK agy)
float32 acc_z               # m/s^2 (SDK agz)
float32 height              # m     (SDK h, cm)
float32 tof                 # m     (SDK tof, cm)
float32 baro                # m     (SDK baro)
float32 battery             # %     (SDK bat)
float32 temp_low            # degC  (SDK templ)
float32 temp_high           # degC  (SDK temph)
float32 motor_time          # s     (SDK time)
```

Fields missing from a given firmware are NaN.

`TelloStatus.msg`:

```
std_msgs/Header header
uint8 DISCONNECTED=0
uint8 LANDED=1
uint8 TAKING_OFF=2
uint8 FLYING=3
uint8 LANDING=4
uint8 EMERGENCY=5
uint8 state
string backend              # sdk | sim | fake
bool link_ok
bool hold                   # /tello/hold active
float32 battery             # %, NaN if unknown
string cmd_source           # none | runner | safety
```

`TelloEvent.msg`:

```
std_msgs/Header header
string type                 # e.g. watchdog_hover, mocap_lost_hover, mocap_lost_land, volume_predicted, volume_hard_land,
                            #      battery_land, link_lost, takeoff_refused, emergency, hold_on, hold_off
string detail               # free text / JSON
```

`FlightSegment.msg`:

```
std_msgs/Header header
uint32 index
string type                 # takeoff | goto | sync | excite | arena | hover | land
string name
bool closed_loop            # true for goto, arena
string params_json          # the segment's YAML parameters, serialised
```

### 4.4 Backend interface (`tello_driver.backends.base.Backend`)

```python
class Backend(ABC):
    capabilities: set[str]          # e.g. {"telemetry", "truth"}; checked by tello_node
    def connect(self) -> None: ...  # raises BackendError on failure
    def close(self) -> None: ...
    def send_sticks(self, rc: tuple[int, int, int, int]) -> None: ...  # already limited and rounded, SDK order (a, b, c, d)
    def takeoff(self) -> None: ...  # non-blocking; completion reported through on_event
    def land(self) -> None: ...
    def emergency(self) -> None: ...
    # callbacks set by tello_node before connect()
    on_telemetry: Callable[[TelemetrySample], None]  # TelemetrySample: dataclass, same fields as TelloTelemetry, plus raw: str
    on_event: Callable[[str, str], None]             # (type, detail), e.g. ("takeoff_done", ""), ("reply", "ok")
```

Backends only handle input/output. All policy (limits, safety, state machine, timing) lives in `tello_node` and `tello_core`.

## 5. `tello_node`

### 5.1 Command path

- A 50 Hz timer (parameter `send_rate`) does the following on each tick:
  1. select the source;
  2. apply per-axis limits (`stick_max`, default 0.4 for every axis on real flights; first flights use 0.2);
  3. round to SDK integers;
  4. call `backend.send_sticks`;
  5. publish `/tello/cmd_sent` with the stamp taken just before the call.

  Sticks are sent on every tick, even when they haven't changed, so `cmd_sent` is a regular stream.
- **Source priority:** `safety` (internal overrides) > `hold` (zero sticks) > `runner` (`/tello/cmd`). Gamepad control is future work and would slot in above `runner`.
- Commands are accepted only in the FLYING state. In any other state, zero sticks are sent.

### 5.2 State machine

```
DISCONNECTED → (connect ok + first telemetry) → LANDED
LANDED → takeoff (pre-flight checks pass) → TAKING_OFF → (backend takeoff_done) → FLYING
FLYING → land (service | failsafe | runner) → LANDING → (backend land_done or z < 0.15 m) → LANDED
any → emergency → EMERGENCY (stays until node restart or explicit reset)
any (except EMERGENCY) → link lost → DISCONNECTED (with land request sent first if FLYING)
```

**Pre-flight checks** (takeoff is refused with a `takeoff_refused` event if any fails):
- backend connected;
- battery ≥ `battery_takeoff_min` (30 %);
- Vicon pose valid within the last 0.2 s and inside the soft volume;
- Vicon rate ≥ 90 % of `mocap_expected_rate`;
- recorder running, if `require_recording` is true (the default on real flights).

### 5.3 Safety rules (pure functions in `tello_core.safety`, applied every tick while FLYING)

| Rule | Trigger (defaults are parameters) | Action | Event |
|---|---|---|---|
| Command watchdog | No `/tello/cmd` for 0.3 s | Zero sticks | `watchdog_hover` |
| Command watchdog (long) | No `/tello/cmd` for 5 s | Land | `watchdog_land` |
| Vicon loss | No valid pose for 0.5 s | Zero sticks (overrides runner) | `mocap_lost_hover` |
| Vicon loss (long) | No valid pose for 1.5 s | Land | `mocap_lost_land` |
| Predicted volume | `p + v * T_look` outside the hard box, `T_look = cmd_delay + stop_time` (0.7 + 0.5 s) | Zero sticks (overrides runner) | `volume_predicted` |
| Hard volume | `p` outside the hard box | Land | `volume_hard_land` |
| Battery | battery ≤ 15 % | Land | `battery_land` |
| Link | No telemetry for 1 s | Zero sticks, then land; the Tello also lands by itself after 15 s without commands | `link_lost` |

- **Volumes:** the soft box and hard box are defined in `tello_bringup/config/lab.yaml`. A flight plan may only shrink them. The soft box is used by the runner (section 7.3), the hard box by the node.
- **Velocity `v`** comes from `/tello/mocap/odom`.

### 5.4 Backends

- **`sdk`** (real Tello, SDK text protocol):
  - **Connect:** bind the sockets to the dongle's IP. Send `command` and retry until `ok` (3 attempts, 1 s apart). Query `sn?`, `sdk?` and `battery?`, and save the answers for `flight.yaml`. Listen for state on 0.0.0.0:8890.
  - **Commands:** `send_sticks` → `rc a b c d`. `takeoff` / `land` / `emergency` → the matching command, with completion taken from the `ok` reply (20 s timeout → `error` event).
  - **Telemetry:** state strings are parsed by `tello_core.sdk.parse_state` and stamped on arrival.
- **`sim`** (Gazebo Harmonic, section 10): converts `rc` back into velocity setpoints (`stick_scale`), with an optional pure delay (`sim_cmd_delay`, default 0.6 s). This is separate from `cmd_delay`, the delay that `tello_node` and `flight_runner` *assume* when predicting the position. It emulates takeoff, landing, emergency and telemetry.
- **`fake`** (in-process): first-order dynamics per axis plus a pure delay, using `tello_core.fake`. It publishes `/sim/truth/odom` and emulated telemetry. Wall clock only.

## 6. Motion capture and time

### 6.1 Vicon sources and `mocap_adapter`

There are two Vicon sources, selected with `mocap_source` in `lab.yaml`:

| Source | How | Timestamps | When to use |
|---|---|---|---|
| `lab_node` (primary) | The lab's ROS node on the Vicon computer publishes the pose; we subscribe over Ethernet | Vicon computer's clock, mapped to the laptop offline (section 6.4) | Default |
| `vrpn` (fallback) | `session.launch.py` starts `vrpn_mocap` on the laptop, connected directly to Vicon Tracker's VRPN server (port 3883) | Laptop clock at arrival. The offset is close to 0; only Vicon's constant processing delay remains | When the lab node can't be used: no DDS link with Jazzy, unusable stamps, or a failed `clock_check`. Needs Tracker's VRPN output enabled and the lab's permission |

Everything downstream is identical. Only `input_topic` and `input_type` change (`/vrpn_mocap/<rigid_body>/pose`, `PoseStamped`). The alignment in section 6.4 runs unchanged and simply finds an offset close to zero. `session.yaml` and `flight.yaml` record which source was used.

**`mocap_adapter`:**

- **Parameters:**
  - `input_topic`;
  - `input_type`: `PoseStamped`, `TransformStamped` or `Odometry`;
  - `rigid_body`;
  - `heading_offset`;
  - `expected_rate`.
- **Subscription QoS:** best effort with depth 10, which works with both reliable and best-effort publishers.
- **Validation (online):**
  - drop a sample if the position jumps more than `thr_pos` (0.3 m) from the last valid sample extrapolated with its velocity;
  - drop it if yaw jumps more than `thr_yaw` (0.5 rad);
  - drop it if z is below `z_floor`.

  Thresholds grow while the adapter keeps rejecting, as in Švrčina (`thr * (3 + n_hold) / 3`). Dropped samples are counted in `/tello/status` diagnostics.
- **Outputs:** `/tello/mocap/pose` (validated, source stamp) and `/tello/mocap/odom`. Velocity in the odom output comes from finite differences on source stamps, then a first-order low-pass filter (τ = 50 ms).

### 6.2 `mocap_sim` (sim and fake only)

- Reads `/sim/truth/odom` and publishes on **the same raw topic name and type as the lab Vicon node** (from `lab.yaml`).
- **Parameters:**
  - rate (default 100 Hz);
  - position noise (0.5 mm);
  - yaw noise (0.1°);
  - dropped frames: probability 1 %, bursts up to 5 frames;
  - network delay: 3 ms ± 1 ms jitter;
  - **clock offset: default 0.35 s, plus drift of 20 ppm**, applied to the header stamp to emulate the separate Vicon computer;
  - `stamp_mode`: `source` (normal) or `none` (zero stamps, to test the fallback).
- The values used are written to the bag as `/sim/mocap_sim/config` (std_msgs/String, YAML, transient-local), so tests can compare them with what the builder recovers.

### 6.3 `clock_check`

- Subscribes to the raw Vicon topic. Over a sliding window (default 60 s), it computes:
  - message rate;
  - dropped frames (gaps longer than 1.5 / expected rate);
  - `arrival − stamp` (offset plus delay): the 5th-percentile envelope, its jitter (95th percentile minus 5th percentile), and the drift (linear fit, in ppm);
  - whether the stamps are usable (non-zero and increasing).
- Prints a live line every 2 s.
- On exit, writes `clock_check.yaml` to the session folder.
- **Pass thresholds:** rate within 5 % of expected; drops < 1 %; jitter < 5 ms; stamps usable (if not usable, a warning: the builder will use the fallback).

### 6.4 Offline alignment (in `tello_dataset`, algorithms in `tello_core.sync`)

1. **Vicon → laptop clock:**
   - `d_k = arrival_k − stamp_k`, where arrival is the bag receive time;
   - take the 5th-percentile envelope over 2 s windows and fit `d(t) = a + b·t` robustly (Huber);
   - mapped time: `t_k = stamp_k + a + b·stamp_k`;
   - residual: the Vicon processing delay, constant and a few milliseconds, reported but not corrected.
2. **Fallback when stamps aren't usable:**
   - rebuild the frame index from the arrival intervals (a gap longer than 1.5 T counts as dropped frames);
   - fit `arrival_k ≈ c + T·k` on the lower envelope;
   - `t_k = c + T·k`.
3. **Telemetry delay:**
   - resample Vicon and telemetry to 1 kHz over the flight;
   - cross-correlate yaw rate, vertical speed and height (Vicon z against `h`) over lags from −0.5 to +1.0 s at 1 ms resolution;
   - combine the peaks, weighted by their sharpness;
   - shift the telemetry timestamps by the estimate.
4. **Command-to-motion delay per axis** (a quality metric, not the final identification): fit a first-order model plus delay on the `sync` and `excite` segments, searching the delay from 0 to 1.2 s in 10 ms steps, using `cmd_sent` and heading-frame velocities from Vicon.

## 7. Flight plans and `flight_runner`

### 7.1 Plan format

The plan is YAML, validated against the schema in `tello_core.plan`. Example (`tello_flight/plans/sysid_basic.yaml`):

```yaml
name: sysid_basic
limits: {stick_max: [0.3, 0.3, 0.3, 0.4]}        # may only lower the bringup limits
volume_shrink: 0.0                               # m, shrinks the lab soft/hard boxes
segments:
  - {type: takeoff}
  - {type: goto, target: [0.0, 0.0, 1.2], yaw: 0.0}
  - {type: sync}
  - {type: excite, axes: [x], signal: doublet, amplitude: 0.3, half_period: 1.0}
  - {type: goto, target: [0.0, 0.0, 1.2]}
  - {type: excite, axes: [x], signal: staircase, amplitudes: [0.02, 0.05, 0.08, 0.12, 0.2], hold: 1.0}
  - {type: goto, target: [0.0, 0.0, 1.2]}
  - {type: excite, axes: [x, y, z, yaw], signal: multisine, amplitude: [0.2, 0.2, 0.15, 0.3],
     f_min: 0.1, f_max: 2.0, duration: 20.0, seed: 3}
  - {type: hover, duration: 5.0}
  - {type: arena, duration: 60.0, seed: 1}
  - {type: goto, target: [0.0, 0.0, 1.2]}
  - {type: sync}
  - {type: land}
```

### 7.2 Segment types

| Type | Feedback | Behaviour | Parameters |
|---|---|---|---|
| `takeoff` | – | Calls `/tello/takeoff`; ends when FLYING and Vicon speed < 0.1 m/s for 1 s | `timeout` (20 s) |
| `goto` | Vicon | Gentle position controller acting on the position predicted one delay ahead: `u = clip(Kp·(target − (p + v·cmd_delay)) / stick_scale)`, speed limited | `target`, `yaw`, `kp` (0.8), `v_max` (0.4 m/s), `tol` (0.1 m), `timeout` (15 s) |
| `sync` | none | 2 s hover, yaw doublet (±0.4 for 0.5 s each), 1 s hover, vertical bump (+0.3 then −0.3 for 0.4 s each), 2 s hover | amplitudes and durations; amplitudes may not exceed the plan's stick limits |
| `excite` | none | Generated signal on the chosen axes (others 0) | `signal` ∈ {`step`, `doublet`, `staircase`, `chirp`, `multisine`}; `amplitude(s)`, `duration`/`hold`/`half_period`, `f_min`/`f_max`, `seed` |
| `arena` | Vicon | Random waypoints in the soft box with smoothed excitation (the legacy `Arena` logic, ported to `tello_core`) | `duration`, `seed`, `v_max`, `u_exc` |
| `hover` | none | Zero sticks | `duration` |
| `land` | – | Calls `/tello/land`; ends when LANDED | `timeout` (20 s) |

- **Signal generators** (`tello_core.signals`):
  - step and doublet: rectangular;
  - staircase: increasing small amplitudes, alternating sign, to map the dead zone;
  - chirp: linear sweep;
  - multisine: Schroeder phases, frequencies chosen on separate harmonic grids per axis so the axes don't correlate.
- **Excitation averages to zero** except `step`, which is followed by a `goto` in the example plans. (`staircase` alternates sign, so it averages to zero too.)
- **The validator rejects any amplitude above the plan's `limits.stick_max`** for that axis, including the `sync` amplitudes.

### 7.3 Runner behaviour

- **Commands:** published on `/tello/cmd` at 50 Hz while FLYING. Each segment start is announced on `/tello/flight/segment`.
- **Soft volume during `excite`:** if `p + v·T_look` leaves the soft box, the segment is cut short and a `goto` to the plan's last target is inserted. A `segment_truncated` event is logged.
- **Hold and failsafes:** if `/tello/status` shows `hold` or a failsafe landing, the runner stops the plan and ends.
- **Dry-run mode:** `--dry-run` runs the whole plan offline against `tello_core.fake` and fails if any predicted position leaves the soft box or the plan breaks a limit. `flight.launch.py` runs it automatically before every flight, simulated or real, and refuses to start if it fails.

### 7.4 `safety_console`

- A terminal node (raw keyboard input) launched by `session.launch.py` in its own terminal window.
- **Keys:**
  - **H:** `/tello/hold` true, then `/flight_runner/abort`;
  - **R:** hold false;
  - **Space:** `/tello/land`;
  - **E:** `/tello/emergency`;
  - **T:** takeoff, for manual checks only.
- **Display:** state, battery, Vicon age and rate, active segment and the latest events, at 5 Hz.

## 8. Recording

- **`session.launch.py backend:=<sdk|sim|fake> session:=<YYYY-MM-DD_x>`** starts:
  - `tello_node`, `mocap_adapter`, `clock_check`, `safety_console`;
  - plus `vrpn_mocap` when `lab.yaml` sets `mocap_source: vrpn`;
  - plus Gazebo, `ros_gz_bridge` and `mocap_sim` when `backend:=sim`, or `mocap_sim` alone when `backend:=fake`.

  It writes `session.yaml` (lab config, DDS profile, drone serial and firmware, operator) to `dataset/<source>/raw/<session>/`.
- **`flight.launch.py plan:=<file> [seed:=…] [notes:=…]`** does three things:
  1. runs the dry run;
  2. starts `ros2 bag record` (MCAP, topic list from `tello_bringup/config/record_topics.yaml`, `--use-sim-time` in simulation) into `dataset/<source>/raw/<session>/<flight_id>/bag/`;
  3. starts `flight_runner`.

  When the runner exits, it stops the recorder (SIGINT).
- **Flight ID:** `<session>_<NN>_<plan name>`.
- **Recorded topics:**
  - every `/tello/*` topic listed in section 4.2;
  - the raw Vicon topic;
  - `/sim/truth/odom`, `/sim/mocap_sim/config` and `/sim/backend/config` in simulation;
  - `/rosout`, and `/clock` in simulation.
- **`flight.yaml`**, written by `flight_runner` at start and completed at the end:
  - flight ID, session, date;
  - plan file path and SHA-256, seed;
  - backend, drone serial and firmware;
  - battery at start and end;
  - Vicon source (`lab_node` or `vrpn`), topic, rigid body and marker-layout note;
  - flight volume used;
  - prop guards, payload, floor type;
  - software version (`git describe` when the repository is under git, otherwise `unversioned`);
  - operator notes;
  - safety event count.
- **Raw data is immutable.** The builder never writes into `raw/`.

## 9. Dataset builder (`tello_dataset`)

### 9.1 Command line

```
python -m tello_dataset build dataset/<source>/raw/<session>/<flight_id> --out dataset/<source>/processed/v1.0 [--rate 100]
python -m tello_dataset build-all dataset/<source>/raw --out dataset/<source>/processed/v1.0
python -m tello_dataset manifest dataset/<source>/processed/v1.0
```

It reads MCAP with `mcap` and `mcap-ros2-support` (no ROS install needed) and works deterministically: the same input and the same builder version always give the same output.

### 9.2 Processing steps

1. Read all recorded topics, keeping both the header stamp and the bag receive time.
2. Align Vicon to the laptop clock (section 6.4, step 1 or 2).
3. Validate Vicon offline: the same jump thresholds as the adapter, applied both forward and backward. Gaps of 50 ms or less are interpolated; longer gaps become NaN with `mocap_valid = false`.
4. Unwrap yaw. Estimate the heading offset θ0 as the median of `atan2(v_y, v_x) − yaw` during forward `excite` segments with speed > 0.2 m/s, and apply it.
5. Compute derivatives with Savitzky-Golay (window 0.15 s, order 2): world velocity, acceleration, yaw rate, and heading-frame velocity. Roll and pitch come from the quaternion.
6. Commands: `cmd_sent` and `cmd` (requested), held constant between their stamps.
7. Telemetry: estimate its delay (section 6.4, step 3), shift it, then interpolate. Battery and temperatures are held constant between samples.
8. Resample onto a regular grid (default 100 Hz) covering takeoff to landing.
9. Label each row with segment index, type and `closed_loop`, the flight state, and the `cmd_source`.

### 9.3 Output files

`dataset/<source>/processed/v1.0/`:
- `<flight_id>.parquet`, one row per grid time:

  | Columns | Units |
  |---|---|
  | `t` | s since takeoff, laptop clock (sim: sim time) |
  | `t_abs` | laptop epoch |
  | `segment_index`, `segment_type`, `closed_loop`, `flight_state`, `cmd_source` | – |
  | `cmd_sent_x/y/z/yaw`, `cmd_req_x/y/z/yaw` | normalised |
  | `pos_x/y/z` | m, arena frame |
  | `qw/qx/qy/qz`, `roll/pitch/yaw` | rad; yaw unwrapped and heading-corrected |
  | `vel_x/y/z` | m/s, world |
  | `velh_x/y/z` | m/s, heading frame |
  | `yaw_rate` | rad/s |
  | `acc_x/y/z` | m/s² |
  | `mocap_valid` | bool |
  | `tel_roll/pitch/yaw`, `tel_vel_x/y/z`, `tel_acc_x/y/z`, `tel_height`, `tel_tof`, `tel_baro`, `tel_battery`, `tel_temp_low/high` | as in `TelloTelemetry` |
  | `tel_valid` | bool |

- `<flight_id>.quality.json`:
  - Vicon: offset `a`, drift `b` (ppm), jitter p95, stamp mode used, frames dropped and rejected (%);
  - telemetry: delay, peak sharpness for each signal;
  - heading offset θ0 and its spread;
  - command-to-motion delay and gain for each axis;
  - battery range;
  - safety events;
  - segments, including truncated ones;
  - in simulation, the true injected values and the recovery error.
- `manifest.csv`: one row per flight (ID, source, date, plan, duration, battery range, quality pass flag, the main quality metrics).
- `splits.yaml`: fixed train/val/test lists **by flight**, written by hand or by `tello_dataset split` with a seed. Never split by individual sample.
- `README.md`: the dataset card (signals, units, frames, method, known limitations), generated from a template.

### 9.4 Quality thresholds

A flight is flagged when a threshold fails; it is never deleted automatically.
- Vicon jitter p95 ≤ 5 ms;
- Vicon dropped plus rejected ≤ 2 %;
- telemetry correlation peak sharpness ≥ 0.5 (normalised);
- battery at the end ≥ 20 %;
- no `volume_hard_land`, `mocap_lost_land` or `link_lost` event.

## 10. Simulation (`tello_sim`, `sim` backend)

- **World `arena.sdf`:**
  - size taken from `lab.yaml` (default 6 × 6 × 3 m);
  - textured floor;
  - nets drawn as visible walls;
  - the soft and hard boxes shown as transparent volumes.
- **Run modes:** headless (`gz sim -s -r`) or with the GUI (`gui:=true`).
- **Model `tello.sdf`:**
  - mass 0.087 kg, about 0.10 × 0.09 m footprint;
  - four `gz-sim-multicopter-motor-model-system` motors and one `gz-sim-multicopter-control-system` (`MulticopterVelocityControl`) taking `gz.msgs.Twist` on its command topic;
  - `gz-sim-odometry-publisher-system` at 100 Hz, giving ground truth.
  - The controller gains start with a velocity time constant of about 0.5 s. They're re-tuned after the first identification (future work).
- **Bridge** (`ros_gz_bridge` YAML):
  - `/clock`;
  - `/sim/cmd_vel` (ROS `geometry_msgs/Twist` → the controller's gz command topic);
  - the gz odometry topic → `/sim/truth/odom`;
  - `/sim/enable` (ROS `std_msgs/Bool` → the controller's gz enable topic).

  On the ROS side, every simulation-only topic sits under `/sim/`, so none can collide with `/tello/*`. The exact gz topic names follow from the SDF and are fixed in the implementation plan.
- **`sim` backend:**
  - **Commands:** `rc/100 × stick_scale` gives the velocity setpoint. `stick_scale` defaults to [1.0, 1.0, 1.0, 1.5] (m/s or rad/s per stick unit). The setpoint goes through a FIFO delay of `sim_cmd_delay` (0.6 s), then is published on `/sim/cmd_vel`.
  - **`takeoff`:** enable the controller, climb at 0.4 m/s until z ≥ 0.8 m, then report `takeoff_done`.
  - **`land`:** descend at 0.3 m/s until z ≤ 0.1 m, then disable the controller and report `land_done`.
  - **`emergency`:** disable the controller.
  - **Telemetry:** emulated at 10 Hz from ground truth, rounded like the SDK (deg, dm/s, cm, 0.001 g), with a linear battery drain (100 → 20 % over 7 min of flight) and an emulated arrival delay (default 40 ms ± 10 ms), so the builder's telemetry alignment gets tested.
  - **Injected values recorded:** `sim_cmd_delay`, `stick_scale` and the telemetry delay and jitter are published once on `/sim/backend/config` (std_msgs/String, YAML, transient-local), so tests can compare them with what the builder recovers.
- **Time:** `use_sim_time:=true` for every node and the recorder. Interactive sessions run at a real-time factor of about 1. `flight.launch.py batch:=true` may run faster than real time (`gz sim` with a faster update rate), because every timestamp is in sim time.

## 11. Testing and hardware bring-up

### 11.1 Automated tests

- **`tello_core` and `tello_dataset` (pytest, no ROS):**
  - stick conversion and sign table;
  - SDK rounding;
  - `parse_state` on captured strings: fixtures from bring-up step 1, plus SDK 1.3 and 2.0 examples;
  - safety rules and the state machine, as table-driven tests;
  - signal generators: zero mean where required, amplitudes, multisine frequency grids;
  - plan schema validation and the dry run;
  - sync algorithms on synthetic data with known offset, drift, jitter and drops: offset error < 1 ms, drift error < 5 ppm, the fallback path, and telemetry delay recovered within 10 ms at 10 Hz on synthetic signals;
  - the builder on a small recorded fake bag stored as a test fixture.
- **Integration (`launch_testing`, `backend:=fake`):**
  - the full chain produces a bag, the builder turns it into Parquet, and the fake's delay and gains are recovered within 10 %;
  - kill `mocap_sim` and the drone lands within 1.5 s;
  - stop the runner and it hovers within 0.3 s;
  - push the drone towards the boundary and `volume_predicted` fires before the hard box;
  - battery rules;
  - the console's services.
- **Protocol (`backend:=sdk` against `fake_tello_udp`):**
  - connect with retries;
  - `rc` format and rate;
  - takeoff and land replies, including a lost reply ending in a timeout `error` event;
  - telemetry dropout leading to `link_lost`.
- **Simulation end-to-end** (success criterion 1): `sysid_basic.yaml` flown in Harmonic with the default `mocap_sim` offset and drift injected. Recovery is checked against `/sim/mocap_sim/config` and `/sim/backend/config`.

### 11.2 Hardware bring-up (in order, each step needs the previous one)

1. **Bench, at home, drone powered but not flying:**
   - connect through the dongle;
   - `sn?`, `sdk?`, `battery?`;
   - measure the telemetry rate;
   - send `rc 0 0 0 0` at 50 Hz for 60 s and check that the drone stays responsive (if not, set `send_rate` to 30 Hz);
   - capture 5 minutes of raw state strings as parser fixtures.
2. **Lab, not flying:**
   - check that the lab network isn't 192.168.10.0/24 and that the firewall is open;
   - DDS discovery over Ethernet;
   - identify the Vicon node: topic, type, rate, stamp source, ROS version;
   - fill in `lab.yaml`;
   - run `clock_check` for 5 minutes and save the report;
   - if the lab node fails any of these checks, switch to the `vrpn` fallback (`mocap_source: vrpn`, Tracker IP in `lab.yaml`) and run `clock_check` again;
   - **hand-carry test:** move the powered drone by hand inside the volume (yaw turns, lifts, translations) while recording. Check the arena frame (z up, origin), the rigid-body heading against the drone's nose, the telemetry axis and sign mapping, and the telemetry-to-Vicon cross-correlation.
3. **First flights:**
   - `stick_max` 0.2;
   - takeoff, hover, land;
   - sign check with ±0.1 on each axis in turn;
   - then `sync` alone;
   - then `sysid_basic.yaml` at reduced amplitude.

   A second person watches with the console (Space and E). Prop guards stay on.

## 12. Low-level protocol bench test

Time-boxed to 2 days. The code is throwaway and lives in `experiments/lowlevel_spike/`, outside `src/`.

- **Question:** does the unofficial protocol (the one the Tello app uses, reverse-engineered by libraries such as TelloPy) give data worth adding a `lowlevel` backend for?
- **Method:** on the bench, with the drone not flying, connect with an existing library and record for 5 minutes. Measure:
  - the message types and their rates;
  - whether there's an onboard counter or timestamp;
  - IMU (gyro, accelerometer), attitude, and the drone's own velocity estimate;
  - battery voltage;
  - stick resolution;
  - firmware compatibility;
  - how switching between SDK mode and this mode behaves (whether the drone must be restarted).
- **Go if both hold:**
  - (a) gyro or attitude arrives at ≥ 50 Hz with an onboard counter or timestamp, or the protocol provides a state signal the SDK lacks that matters for the model;
  - (b) it works reliably on our firmware.
- **Output:** `experiments/lowlevel_spike/REPORT.md` with the measurements and the decision. A go becomes a new design addition (a `lowlevel` backend behind the same interface). A no-go changes nothing.

## 13. Risks and open questions

| Risk / unknown | Impact | Mitigation |
|---|---|---|
| Vicon node details unknown (topic, type, stamps, ROS version) | Adapter configuration, sync | Configurable adapter (three message types); `clock_check`; fallback stamp mode; `vrpn` fallback source; checklist at bring-up step 2 |
| Different ROS versions between the Vicon computer and Jazzy | No Vicon data | Usually works for standard messages. If not, use the `vrpn` fallback (section 6.1), which needs no DDS link to the Vicon computer. Last resort: a small relay in a distrobox container running the Vicon computer's ROS version, forwarding poses over a plain UDP socket |
| The Tello rejects `rc` at 50 Hz | Command regularity | Bench step 1; `send_rate` parameter (fallback 30 Hz) |
| SDK telemetry axis and sign meaning unclear | Wrong telemetry columns | Hand-carry test; raw strings recorded so they can be re-parsed |
| Noisy Vicon heading on an 87 g drone | Heading error | Four or more markers in an asymmetric layout; jump filter; θ0 estimated per flight |
| Lab network on 192.168.10.0/24 | Routing clash | Checked at bring-up step 2; ask the lab for a different network, or route the Tello subnet explicitly |
| Delay of 0.6 s taken from a single thesis | Simulation realism, safety lookahead | Parameters; replaced by the measured value after the first flights |
| Floor texture or lighting affects the Tello's position hold | Data variation | Recorded in `flight.yaml`; kept constant within a session |

## 14. Future work (not in v1)

- `track` segments with a delay-compensated tracking controller, after the first identification flights
- video recording (`streamon`, H.264 decoding, frame timestamps)
- a gamepad as a control source with deadman priority
- a `lowlevel` backend, depending on section 12
- multiple drones
- re-tuning the Harmonic model from identified parameters
- the online twin, which consumes `/tello/cmd_sent` and `/tello/mocap/*`
