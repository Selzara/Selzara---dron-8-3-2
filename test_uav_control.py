import unittest
from unittest.mock import MagicMock

import pytest
from pymavlink import mavutil

from uav_control import UAVControl


def test_land_success(self):
    mock_master = MagicMock()
    mock_master.mode_mapping.return_value = {'LAND': 9}
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master
    uav.wait_command_ack = MagicMock(return_value=True)

    uav.land()

    mock_master.set_mode.assert_called_with(9)
    uav.wait_command_ack.assert_called_with(mavutil.mavlink.MAV_CMD_NAV_LAND)
    assert uav.wait_command_ack.call_count == 1


def test_land_command_ack_failure(self):
    mock_master = MagicMock()
    mock_master.mode_mapping.return_value = {'LAND': 9}
    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master
    uav.wait_command_ack = MagicMock(return_value=False)

    with pytest.raises(RuntimeError, match="Failed to land: Команда посадки не подтверждена"):
        uav.land()

def test_get_telemetry_vfr_hud(self):
    mock_msg = MagicMock()
    mock_msg.get_type.return_value = 'VFR_HUD'
    mock_msg.groundspeed = 15.0
    mock_msg.airspeed = 16.0
    mock_msg.heading = 90

    mock_master = MagicMock()
    mock_master.recv_match.return_value = mock_msg

    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    telemetry = uav.get_telemetry()
    assert telemetry == {
        'groundspeed': 15.0,
        'airspeed': 16.0,
        'heading': 90
    }

def test_get_telemetry_sys_status(self):
    mock_msg = MagicMock()
    mock_msg.get_type.return_value = 'SYS_STATUS'
    mock_msg.voltage_battery = 11000  # в мВ
    mock_msg.battery_remaining = 80  # в процентах

    mock_master = MagicMock()
    mock_master.recv_match.return_value = mock_msg

    uav = UAVControl.__new__(UAVControl)
    uav.master = mock_master

    telemetry = uav.get_telemetry()
    assert telemetry == {
        'battery_voltage': 11.0,
        'battery_remaining': 80
    }

