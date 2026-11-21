import pytest

from tello_driver.backends import BackendError, FakeBackend, FakeBackendParams, make_backend


class ManualClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def connected(params=FakeBackendParams()):
    clock = ManualClock()
    be = FakeBackend(clock, params)
    got = {"tel": [], "truth": [], "events": []}
    be.on_telemetry = lambda s: got["tel"].append((clock.t, s))
    be.on_truth = lambda tr: got["truth"].append((clock.t, tr))
    be.on_event = lambda k, d: got["events"].append(k)
    be.connect()
    return be, clock, got


def advance(be, clock, seconds, dt=0.005):
    for _ in range(int(round(seconds / dt))):
        clock.t = round(clock.t + dt, 9)
        be.update()


def test_connect_reports_and_streams_telemetry_and_truth():
    be, clock, got = connected()
    advance(be, clock, 2.0)
    assert got["events"] == ["connected"]
    assert 19 <= len(got["tel"]) <= 21 and 199 <= len(got["truth"]) <= 201
    assert got["tel"][0][1].battery == 100.0 and got["tel"][0][1].raw.startswith("pitch:")


def test_telemetry_arrives_after_the_emulated_delay():
    be, clock, got = connected(FakeBackendParams(telemetry_delay=0.04, telemetry_jitter=0.0))
    advance(be, clock, 0.035)
    assert got["tel"] == []
    advance(be, clock, 0.01)
    assert len(got["tel"]) == 1


def test_takeoff_completes_and_sticks_move_the_drone():
    be, clock, got = connected()
    be.takeoff()
    advance(be, clock, 4.0)
    assert "takeoff_done" in got["events"]
    be.send_sticks((0, 50, 0, 0))
    advance(be, clock, 3.0)
    assert got["truth"][-1][1].p[0] > 0.5


def test_commands_before_connect_raise():
    be = FakeBackend(ManualClock())
    with pytest.raises(BackendError):
        be.send_sticks((0, 0, 0, 0))


def test_make_backend():
    be = make_backend("fake", ManualClock(), {"delay": 0.3, "battery_start": 50.0, "start": [1.0, 0.0, 0.0]})
    assert be.sim_config()["cmd_delay"] == 0.3 and be.prm.start == (1.0, 0.0, 0.0)
    assert be.info()["serial"] == "FAKE0001"
    with pytest.raises(BackendError, match="Plan 4"):
        make_backend("sdk", ManualClock())
    with pytest.raises(BackendError, match="unknown backend"):
        make_backend("gazebo", ManualClock())
