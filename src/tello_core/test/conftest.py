import pytest

from tello_core.fake import FakeDynamics, FakeParams


@pytest.fixture
def flying_fake():
    """make(position, yaw, params) -> (fake that has finished taking off, current time)."""
    def make(position=(0.0, 0.0, 0.0), yaw=0.0, params=FakeParams()):
        f = FakeDynamics(params, position=position, yaw=yaw, t0=0.0)
        f.takeoff(0.0)
        t = 0.0
        while f.mode != "flying":
            t = round(t + 0.005, 9)
            f.step(t)
            if t > 10.0:
                raise AssertionError("fake did not take off")
        return f, t
    return make
