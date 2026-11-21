import math

import pytest

from tello_core.drone import DroneConfig, DroneCore
from tello_core.flight_state import FlightState as FS
from tello_core.flight_state import PreflightConfig
from tello_core.safety import Box, SafetyConfig
from tello_core.sdk import TelemetrySample
from tello_core.sticks import ZERO, Sticks

SOFT = Box(-2.0, 2.0, -2.0, 2.0, 0.5, 2.0)
HARD = Box(-2.5, 2.5, -2.5, 2.5, 0.2, 2.5)


def make(**kw):
    return DroneCore(DroneConfig(safety=SafetyConfig(hard_box=HARD), preflight=PreflightConfig(soft_box=SOFT), **kw))


class Rig:
    """Feeds a DroneCore on a 100 Hz grid: poses every step, telemetry every 0.1 s, ticks at 50 Hz."""

    def __init__(self, core):
        self.core, self.t, self.outs = core, 0.0, []

    def run(self, duration, pose=(0.0, 0.0, 0.0), v=(0.0, 0.0, 0.0), cmd=None, telemetry=True, poses=True,
            battery=80.0):
        for _ in range(int(round(duration / 0.01))):
            self.t = round(self.t + 0.01, 6)
            if poses:
                self.core.on_pose(self.t, pose, 0.0, v)
            if telemetry and abs(self.t * 10 - round(self.t * 10)) < 1e-6:
                self.core.on_telemetry(self.t, TelemetrySample(battery=battery))
            if cmd is not None:
                self.core.on_cmd(self.t, cmd)
            if abs(self.t * 50 - round(self.t * 50)) < 1e-6:
                self.outs.append(self.core.tick(self.t))
        return self.outs[-1]

    def events(self):
        return [e for o in self.outs for e, _ in o.events]

    def actions(self):
        return [a for o in self.outs for a in o.actions]


def ready_rig(**kw):
    core = make(**kw)
    core.on_connected(0.0)
    core.set_recording(True)
    rig = Rig(core)
    rig.run(1.0)
    return rig


def flying_rig(**kw):
    rig = ready_rig(**kw)
    ok, msg = rig.core.request_takeoff(rig.t)
    assert ok, msg
    rig.core.on_backend_event(rig.t, "takeoff_done")
    rig.run(0.1, pose=(0.0, 0.0, 1.0), cmd=ZERO)
    return rig


def test_disconnected_until_first_telemetry():
    core = make()
    assert core.tick(0.0).status.state == FS.DISCONNECTED
    core.on_telemetry(0.1, TelemetrySample(battery=90.0))
    assert core.tick(0.12).status.state == FS.LANDED


def test_takeoff_refused_without_vicon_and_event_raised():
    core = make()
    core.on_connected(0.0)
    core.set_recording(True)
    core.on_telemetry(0.1, TelemetrySample(battery=90.0))
    ok, msg = core.request_takeoff(0.2)
    assert not ok and "no recent Vicon pose" in msg
    assert ("takeoff_refused", msg) in core.tick(0.22).events


def test_takeoff_refused_without_recording():
    rig = ready_rig()
    rig.core.set_recording(False)
    ok, msg = rig.core.request_takeoff(rig.t)
    assert not ok and "recorder not running" in msg


def test_takeoff_accepted_when_ready_then_flying():
    rig = ready_rig()
    assert rig.outs[-1].status.mocap_rate == pytest.approx(100.0)
    assert rig.core.request_takeoff(rig.t)[0] and rig.core.sm.state == FS.TAKING_OFF
    rig.core.on_backend_event(rig.t, "takeoff_done")
    assert rig.run(0.02, pose=(0.0, 0.0, 1.0)).status.state == FS.FLYING


def test_runner_commands_are_rounded_and_reported_as_sent():
    out = flying_rig().run(0.1, pose=(0.0, 0.0, 1.0), cmd=Sticks(x=0.1234))
    assert out.rc == (0, 12, 0, 0) and out.sent.x == pytest.approx(0.12) and out.status.cmd_source == "runner"


def test_commands_are_clamped_to_bring_up_limits():
    out = flying_rig(stick_max=(0.2, 0.2, 0.2, 0.2)).run(0.1, pose=(0.0, 0.0, 1.0), cmd=Sticks(x=0.5))
    assert out.rc == (0, 20, 0, 0)


def test_watchdog_hovers_then_lands():
    rig = flying_rig()
    out = rig.run(0.4, pose=(0.0, 0.0, 1.0))
    assert "watchdog_hover" in rig.events() and out.status.cmd_source == "safety" and out.rc == (0, 0, 0, 0)
    rig.run(4.8, pose=(0.0, 0.0, 1.0))
    assert "watchdog_land" in rig.events() and "land" in rig.actions() and rig.core.sm.state == FS.LANDING


def test_command_age_counts_from_entering_flying():
    rig = ready_rig()
    rig.core.request_takeoff(rig.t)
    rig.core.on_backend_event(rig.t, "takeoff_done")
    rig.run(0.2, pose=(0.0, 0.0, 1.0))
    assert "watchdog_hover" not in rig.events()
    rig.run(0.2, pose=(0.0, 0.0, 1.0))
    assert "watchdog_hover" in rig.events()


def test_vicon_loss_hovers_then_lands():
    rig = flying_rig()
    rig.run(0.6, cmd=ZERO, poses=False)
    assert "mocap_lost_hover" in rig.events()
    rig.run(1.0, cmd=ZERO, poses=False)
    assert "mocap_lost_land" in rig.events() and rig.core.sm.state == FS.LANDING


def test_invalid_poses_age_out_and_land():
    rig = flying_rig()
    for _ in range(160):
        rig.t = round(rig.t + 0.01, 6)
        assert rig.core.on_pose(rig.t, (math.nan, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0)) is False
        rig.core.on_cmd(rig.t, ZERO)
        if abs(rig.t * 10 - round(rig.t * 10)) < 1e-6:
            rig.core.on_telemetry(rig.t, TelemetrySample(battery=80.0))
        if abs(rig.t * 50 - round(rig.t * 50)) < 1e-6:
            rig.outs.append(rig.core.tick(rig.t))
    assert "mocap_lost_land" in rig.events() and rig.core.sm.state == FS.LANDING


def test_predicted_exit_of_hard_box_hovers():
    rig = flying_rig()
    out = rig.run(0.1, pose=(2.0, 0.0, 1.0), v=(0.5, 0.0, 0.0), cmd=Sticks(x=0.3))   # 2.0 + 0.5 * 1.2 = 2.6
    assert "volume_predicted" in rig.events() and out.rc == (0, 0, 0, 0)


def test_outside_hard_box_lands():
    rig = flying_rig()
    rig.run(0.1, pose=(2.6, 0.0, 1.0), cmd=ZERO)
    assert "volume_hard_land" in rig.events() and "land" in rig.actions()


def test_low_battery_lands():
    rig = flying_rig()
    rig.run(0.2, pose=(0.0, 0.0, 1.0), cmd=ZERO, battery=14.0)
    assert "battery_land" in rig.events() and rig.core.sm.state == FS.LANDING


def test_link_loss_lands_and_disconnects():
    rig = flying_rig()
    rig.run(1.1, pose=(0.0, 0.0, 1.0), cmd=ZERO, telemetry=False)
    assert "link_lost" in rig.events() and "land" in rig.actions() and rig.core.sm.state == FS.DISCONNECTED


def test_hold_overrides_runner_until_released():
    rig = flying_rig()
    rig.core.set_hold(rig.t, True)
    out = rig.run(0.1, pose=(0.0, 0.0, 1.0), cmd=Sticks(x=0.3))
    assert out.status.cmd_source == "hold" and out.rc == (0, 0, 0, 0) and "hold_on" in rig.events()
    rig.core.set_hold(rig.t, False)
    out = rig.run(0.1, pose=(0.0, 0.0, 1.0), cmd=Sticks(x=0.3))
    assert out.status.cmd_source == "runner" and out.rc == (0, 30, 0, 0) and "hold_off" in rig.events()


def test_safety_beats_hold():
    rig = flying_rig()
    rig.core.set_hold(rig.t, True)
    rig.run(0.1, pose=(2.6, 0.0, 1.0), cmd=ZERO)
    assert "volume_hard_land" in rig.events() and rig.core.sm.state == FS.LANDING


def test_emergency_is_sticky_until_reset():
    rig = flying_rig()
    rig.core.emergency(rig.t)
    out = rig.run(0.1, pose=(0.0, 0.0, 0.5), cmd=Sticks(x=0.3))
    assert out.status.state == FS.EMERGENCY and out.rc == (0, 0, 0, 0) and "emergency" in rig.events()
    assert rig.core.request_takeoff(rig.t)[0] is False
    assert rig.core.reset(rig.t)
    assert rig.run(0.2).status.state == FS.LANDED


def test_landing_completes_on_the_backend_report():
    rig = flying_rig()
    assert rig.core.request_land(rig.t)[0]
    rig.core.on_backend_event(rig.t, "land_done")
    assert rig.run(0.02, pose=(0.0, 0.0, 0.05)).status.state == FS.LANDED


def test_landing_falls_back_to_a_settled_vicon_height():
    rig = flying_rig()
    assert rig.core.request_land(rig.t)[0]
    rig.run(0.5, pose=(0.0, 0.0, 0.1))
    assert rig.core.sm.state == FS.LANDING              # low, but not for land_settle (1 s) yet
    rig.run(0.6, pose=(0.0, 0.0, 0.1))
    assert rig.core.sm.state == FS.LANDED


def test_events_are_reported_once_while_active():
    rig = flying_rig()
    rig.run(1.0, pose=(0.0, 0.0, 1.0))
    assert rig.events().count("watchdog_hover") == 1


def test_status_changed_only_on_key_change():
    rig = ready_rig()
    assert rig.outs[0].status_changed and not rig.outs[-1].status_changed
    rig.core.set_hold(rig.t, True)
    assert rig.run(0.02).status_changed


def test_command_from_before_takeoff_is_ignored():
    rig = ready_rig()
    rig.core.on_cmd(rig.t, Sticks(x=0.3))
    rig.core.request_takeoff(rig.t)
    rig.t = round(rig.t + 0.01, 6)
    rig.core.on_backend_event(rig.t, "takeoff_done")
    out = rig.run(0.1, pose=(0.0, 0.0, 1.0))
    assert out.rc == (0, 0, 0, 0) and out.status.cmd_source == "none"


def test_config_validation():
    with pytest.raises(ValueError):
        make(stick_max=(0.4, 0.4, 0.4, 1.5))
    with pytest.raises(ValueError):
        make(mocap_expected_rate=0.0)
