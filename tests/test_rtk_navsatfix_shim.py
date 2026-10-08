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
        """Store a callback for manual invocation and track cancellation."""
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        """Record cancellation without scheduling or invoking the callback."""
        self.cancelled = True


class _Logger:
    def __init__(self):
        """Initialize an ordered record of log levels and messages."""
        self.messages = []

    def info(self, msg):
        """Record an informational message for later assertions."""
        self.messages.append(('info', msg))

    def warn(self, msg):
        """Record a warning message for later assertions."""
        self.messages.append(('warn', msg))


class _Publisher:
    def __init__(self):
        """Initialize the collection of published message references."""
        self.published = []

    def publish(self, msg):
        """Retain the message reference in publication order without copying it."""
        self.published.append(msg)


class _Node:
    """Minimal rclpy.node.Node double that records timers, publishers and callbacks."""

    def __init__(self, name):
        """Initialize passive ROS API doubles without creating a real node."""
        self.timers = []
        self.publishers = []
        self.callbacks = {}
        self.logger = _Logger()

    def create_publisher(self, msg_type, topic, qos):
        """Return and record a publisher double, ignoring ROS metadata."""
        publisher = _Publisher()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        """Store the callback by topic for direct invocation in tests."""
        self.callbacks[topic] = callback

    def create_timer(self, period, callback):
        """Return and record a timer double without scheduling its callback."""
        timer = _Timer(callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        """Return the node's shared log recorder."""
        return self.logger


class _NavSatStatus:
    STATUS_NO_FIX = -1
    STATUS_FIX = 0
    STATUS_SBAS_FIX = 1
    STATUS_GBAS_FIX = 2

    def __init__(self):
        """Initialize fix-status and service fields to zero."""
        self.status = 0
        self.service = 0


class _NavSatFix:
    def __init__(self, status=_NavSatStatus.STATUS_GBAS_FIX, latitude=0.0):
        """Build a fix double with configurable status and latitude."""
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
    """Return status codes from the first publisher in publication order."""
    return [msg.status.status for msg in node.publishers[0].published]


def _pvt(carr_soln):
    """Wrap a carrier solution value in a minimal PVT message double."""
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


@pytest.mark.parametrize('fix_count', [0, 1, 100, 101])
@pytest.mark.parametrize('release', ['timeout', 'pvt'])
def test_buffer_boundary_flushes_once_in_arrival_order(shim, fix_count, release):
    """Retain exactly the newest 100 fixes on either cold-start release path."""
    node = shim.node
    for index in range(fix_count):
        node.callbacks['/rover/fix'](shim.fix(latitude=float(index)))
    assert node.publishers[0].published == []
    if release == 'timeout':
        _fire_pvt_timeout(node)
        expected_status = _NavSatStatus.STATUS_NO_FIX
    else:
        node.callbacks['/rover/ubx_nav_pvt'](_pvt(1))
        expected_status = _NavSatStatus.STATUS_SBAS_FIX
    published = node.publishers[0].published
    retained = list(range(max(0, fix_count - 100), fix_count))
    assert [msg.latitude for msg in published] == retained
    assert _statuses(node) == [expected_status] * len(retained)
    assert not node._held_fixes

    node.callbacks['/rover/ubx_nav_pvt'](_pvt(2))
    _fire_pvt_timeout(node)
    node.callbacks['/rover/fix'](shim.fix(latitude=-1.0))
    assert [msg.latitude for msg in published] == [*retained, -1.0]
    assert _statuses(node) == [expected_status] * len(retained) + [2]
    assert node._pub_count == len(retained) + 1
    assert not node._held_fixes


@pytest.mark.parametrize('held_before_timeout', [True, False])
def test_no_fix_forwarding_preserves_payload_without_mutating_input(shim, held_before_timeout):
    """Only status changes when a held or newly received fix is forwarded after timeout."""
    node = shim.node
    fix = shim.fix(latitude=51.5)
    fix.header = types.SimpleNamespace(frame_id='gps', stamp=123)
    fix.longitude = -2.5
    fix.altitude = 42.0
    fix.position_covariance = [4.0, 0.1, 0.2, 0.1, 5.0, 0.3, 0.2, 0.3, 6.0]
    fix.position_covariance_type = 3
    fix.status.service = 5
    if held_before_timeout:
        node.callbacks['/rover/fix'](fix)
    _fire_pvt_timeout(node)
    if not held_before_timeout:
        node.callbacks['/rover/fix'](fix)

    published, = node.publishers[0].published
    assert published is not fix
    assert published.status is not fix.status
    assert published.status.status == _NavSatStatus.STATUS_NO_FIX
    assert fix.status.status == _NavSatStatus.STATUS_GBAS_FIX
    assert published.status.service == fix.status.service
    for field in (
        'header', 'latitude', 'longitude', 'altitude',
        'position_covariance', 'position_covariance_type',
    ):
        assert getattr(published, field) == getattr(fix, field)
    assert node._pub_count == 1
    assert not node._held_fixes


@pytest.mark.parametrize(('carr_soln', 'expected_status'), [(0, 0), (1, 1), (2, 2)])
def test_late_wrapped_pvt_restores_receiver_quality(shim, carr_soln, expected_status):
    """Recovery after timeout supports every PVT quality and the driver's wrapped enum."""
    node = shim.node
    _fire_pvt_timeout(node)
    node.callbacks['/rover/fix'](shim.fix())
    node.callbacks['/rover/ubx_nav_pvt'](_pvt(types.SimpleNamespace(carr_soln=carr_soln)))
    node.callbacks['/rover/fix'](shim.fix(status=_NavSatStatus.STATUS_FIX))
    assert _statuses(node) == [-1, expected_status]
    assert not node._held_fixes
