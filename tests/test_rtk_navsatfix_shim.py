"""rtk_navsatfix_shim must keep forwarding fixes when UBXNavPVT never arrives."""

import importlib.util
import sys
import types
from pathlib import Path

import pytest

SHIM = (Path(__file__).parents[1] / 'src' / 'devkit_driver' / 'devkit_driver'
        / 'rtk_navsatfix_shim.py')


class _Timer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(('info', msg))

    def warn(self, msg):
        self.messages.append(('warn', msg))


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Node:
    """Minimal rclpy.node.Node double that records timers, publishers and callbacks."""

    def __init__(self, name):
        self.timers = []
        self.publishers = []
        self.callbacks = {}
        self.logger = _Logger()

    def create_publisher(self, msg_type, topic, qos):
        publisher = _Publisher()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback

    def create_timer(self, period, callback):
        timer = _Timer(callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self.logger


class _NavSatStatus:
    STATUS_NO_FIX = -1
    STATUS_FIX = 0
    STATUS_SBAS_FIX = 1
    STATUS_GBAS_FIX = 2

    def __init__(self):
        self.status = 0
        self.service = 0


class _NavSatFix:
    def __init__(self, status=_NavSatStatus.STATUS_GBAS_FIX, latitude=0.0):
        self.header = None
        self.latitude = latitude
        self.longitude = 0.0
        self.altitude = 0.0
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = 0
        self.status = _NavSatStatus()
        self.status.status = status


class _Enum:
    BEST_EFFORT = RELIABLE = VOLATILE = 0


@pytest.fixture
def shim(monkeypatch):
    """Load the real shim module against passive ROS doubles and return a new node."""
    modules = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.qos': types.ModuleType('rclpy.qos'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
        'ublox_ubx_msgs': types.ModuleType('ublox_ubx_msgs'),
        'ublox_ubx_msgs.msg': types.ModuleType('ublox_ubx_msgs.msg'),
    }
    modules['rclpy.node'].Node = _Node
    modules['rclpy.qos'].QoSProfile = lambda **kwargs: kwargs
    modules['rclpy.qos'].DurabilityPolicy = _Enum
    modules['rclpy.qos'].ReliabilityPolicy = _Enum
    modules['sensor_msgs.msg'].NavSatFix = _NavSatFix
    modules['sensor_msgs.msg'].NavSatStatus = _NavSatStatus
    modules['ublox_ubx_msgs.msg'].UBXNavPVT = type('UBXNavPVT', (), {})
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('rtk_navsatfix_shim_under_test', SHIM)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.RtkNavSatFixShim()
    return types.SimpleNamespace(module=module, node=node, fix=_NavSatFix)


def _fire_pvt_timeout(node):
    """Run the PVT-timeout timer callback (the first timer the node creates)."""
    timer = node.timers[0]
    timer.callback()
    assert timer.cancelled


def _statuses(node):
    return [msg.status.status for msg in node.publishers[0].published]


def _pvt(carr_soln):
    return types.SimpleNamespace(carr_soln=carr_soln)


def test_fixes_are_held_until_the_timeout(shim):
    """Verify fixes are buffered, not published, while waiting for the first PVT."""
    for _ in range(3):
        shim.node.callbacks['/rover/fix'](shim.fix())
    assert shim.node.publishers[0].published == []
    _fire_pvt_timeout(shim.node)
    assert _statuses(shim.node) == [-1] * 3


def test_fixes_after_pvt_timeout_are_forwarded_as_no_fix(shim):
    """Verify held and later fixes are published as NO_FIX when PVT never arrives."""
    node = shim.node
    for _ in range(3):
        node.callbacks['/rover/fix'](shim.fix())
    _fire_pvt_timeout(node)
    assert _statuses(node) == [-1] * 3
    for _ in range(100):
        node.callbacks['/rover/fix'](shim.fix(latitude=51.0))
    assert _statuses(node) == [-1] * 103
    assert node.publishers[0].published[-1].latitude == 51.0


def test_late_pvt_resumes_corrected_status(shim):
    """Verify the first PVT after a timeout switches output back to carr_soln status."""
    node = shim.node
    _fire_pvt_timeout(node)
    node.callbacks['/rover/fix'](shim.fix())
    node.callbacks['/rover/ubx_nav_pvt'](_pvt(2))
    node.callbacks['/rover/fix'](shim.fix(status=_NavSatStatus.STATUS_FIX))
    assert _statuses(node) == [-1, 2]


def test_pvt_before_timeout_flushes_held_fixes_corrected(shim):
    """Verify the normal cold-start path still flushes held fixes with carr_soln applied."""
    node = shim.node
    node.callbacks['/rover/fix'](shim.fix())
    node.callbacks['/rover/ubx_nav_pvt'](_pvt(1))
    assert _statuses(node) == [1]
    _fire_pvt_timeout(node)
    node.callbacks['/rover/fix'](shim.fix())
    assert _statuses(node) == [1, 1]


def test_held_buffer_is_bounded(shim):
    """Verify only the newest fixes survive a long wait, up to the buffer limit."""
    limit = shim.module._MAX_HELD_FIXES
    for i in range(limit * 3):
        shim.node.callbacks['/rover/fix'](shim.fix(latitude=float(i)))
    _fire_pvt_timeout(shim.node)
    published = shim.node.publishers[0].published
    assert [msg.latitude for msg in published] == [float(i) for i in range(limit * 2, limit * 3)]
