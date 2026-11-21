import os

from tello_driver.testing import isolate_domain


def test_isolate_domain_picks_a_local_only_domain_below_the_ephemeral_ports(monkeypatch):
    # monkeypatch restores both variables after the test, so the launch tests of this process keep theirs
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    monkeypatch.delenv("ROS_AUTOMATIC_DISCOVERY_RANGE", raising=False)
    seen = set()
    for _ in range(500):
        domain = isolate_domain()
        assert 1 <= domain <= 101 and os.environ["ROS_DOMAIN_ID"] == str(domain)
        seen.add(domain)
    assert len(seen) > 50                                    # random, not fixed
    assert os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"
