import math

import pytest

from tello_core.sdk import G, TelemetrySample, format_state, parse_state

STD = ("pitch:1;roll:-2;yaw:90;vgx:5;vgy:-3;vgz:2;templ:60;temph:62;tof:110;h:100;bat:87;"
       "baro:156.81;time:12;agx:-6.00;agy:1.00;agz:-999.00;\r\n")
SDK2 = ("mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:70;temph:72;"
        "tof:10;h:0;bat:55;baro:20.12;time:0;agx:0.00;agy:0.00;agz:-1000.00;\r\n")


def test_parse_standard_tello():
    t = parse_state(STD)
    assert t.pitch == pytest.approx(math.radians(1))
    assert t.roll == pytest.approx(math.radians(-2))
    assert t.yaw == pytest.approx(-math.pi / 2)        # SDK clockwise -> ROS counter-clockwise
    assert (t.vel_x, t.vel_y, t.vel_z) == pytest.approx((0.5, 0.3, -0.2))
    assert t.acc_z == pytest.approx(999 * G / 1000)
    assert (t.height, t.tof) == pytest.approx((1.0, 1.1))
    assert (t.battery, t.baro, t.temp_low, t.temp_high, t.motor_time) == pytest.approx((87, 156.81, 60, 62, 12))
    assert t.raw == STD.strip()


def test_parse_sdk2_ignores_mission_pad_fields():
    t = parse_state(SDK2)
    assert t.battery == 55 and t.height == 0.0


def test_missing_fields_are_nan():
    t = parse_state("bat:50;")
    assert t.battery == 50 and math.isnan(t.yaw) and math.isnan(t.height)


def test_garbage_is_tolerated():
    assert math.isnan(parse_state("ok").battery)
    assert math.isnan(parse_state("").height)
    t = parse_state("bat:abc;h:20;")
    assert math.isnan(t.battery) and t.height == pytest.approx(0.2)


def test_format_then_parse_rounds_like_the_sdk():
    src = TelemetrySample(roll=0.0, pitch=0.0, yaw=0.3, vel_x=0.47, vel_y=-0.12, vel_z=0.0,
                          acc_x=0.0, acc_y=0.0, acc_z=G, height=0.834, tof=0.934, baro=10.0,
                          battery=80.4, temp_low=60, temp_high=62, motor_time=3.2)
    t = parse_state(format_state(src))
    assert t.height == pytest.approx(0.83)          # cm resolution
    assert t.tof == pytest.approx(0.93)
    assert t.vel_x == pytest.approx(0.5)            # dm/s resolution
    assert t.vel_y == pytest.approx(-0.1)
    assert t.yaw == pytest.approx(math.radians(17))  # 0.3 rad = 17.2 deg -> 17 deg
    assert t.acc_z == pytest.approx(G)
    assert t.battery == 80


def test_format_wraps_yaw():
    t = parse_state(format_state(TelemetrySample(yaw=2 * math.pi + 0.1)))
    assert t.yaw == pytest.approx(math.radians(6))
