"""Robot brain commands and row discovery through their domain, application and view-model layers."""
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from devkit_ui.application_services.robot_brain_application_service import RobotBrainApplicationService
from devkit_ui.application_services.row_discovery_application_service import (
    RowDiscoveryApplicationService,
)
from devkit_ui.view_models.run_view_model import RunViewModel


class FakeEmpty:
    pass


class FakeString:
    def __init__(self, data=''):
        """Store the string payload delivered to discovery status callbacks."""
        self.data = data


class FakeTrigger:
    class Request:
        pass


_rclpy_node = ModuleType('rclpy.node')
_rclpy_node.Node = type('Node', (), {})
_rclpy_qos = ModuleType('rclpy.qos')
_rclpy_qos.QoSProfile = lambda **_kwargs: object()
_rclpy_qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT='best_effort')
_std_msgs = ModuleType('std_msgs.msg')
_std_msgs.Empty = FakeEmpty
_std_msgs.String = FakeString
_std_srvs = ModuleType('std_srvs.srv')
_std_srvs.Trigger = FakeTrigger

with patch.dict(sys.modules, {
    'rclpy': ModuleType('rclpy'),
    'rclpy.node': _rclpy_node,
    'rclpy.qos': _rclpy_qos,
    'std_msgs': ModuleType('std_msgs'),
    'std_msgs.msg': _std_msgs,
    'std_srvs': ModuleType('std_srvs'),
    'std_srvs.srv': _std_srvs,
}):
    from devkit_ui.domain_services import row_discovery_domain_service as discovery_module
    from devkit_ui.domain_services.robot_brain_domain_service import ROBOT_BRAIN_COMMANDS, RobotBrainDomainService
    from devkit_ui.domain_services.row_discovery_domain_service import (
        RowDiscoveryDomainService,
    )
    from devkit_ui.ros_gateway import RosGateway


class ImmediateThread:
    def __init__(self, target, daemon=False):  # pylint: disable=unused-argument
        """Capture the worker for deterministic execution on start."""
        self.target = target

    def start(self):
        """Run the captured worker synchronously for deterministic service tests."""
        self.target()


class Future:
    def __init__(self, result=None, error=None):
        """Configure the response or exception returned by the fake service future."""
        self._result = result
        self._error = error

    def result(self):
        """Raise the configured error or return the configured service response."""
        if self._error is not None:
            raise self._error
        return self._result

    def add_done_callback(self, callback):
        """Invoke the completion callback immediately with this future."""
        callback(self)


class TestRobotBrainServices(unittest.TestCase):
    def setUp(self):
        """Wire robot brain services to mock publishers indexed by command topic."""
        self.ros = Mock(spec=RosGateway)
        self.publishers = {}

        def create_publisher(msg_type, topic, _qos):
            """Require empty messages and record a mock publisher for the given topic."""
            self.assertIs(msg_type, FakeEmpty)
            self.publishers[topic] = Mock()
            return self.publishers[topic]

        self.ros.create_publisher.side_effect = create_publisher
        self.domain = RobotBrainDomainService(self.ros)
        self.app = RobotBrainApplicationService(self.domain)

    def test_one_publisher_per_command_on_its_own_topic(self):
        """Verify each supported brain command has a dedicated ESP topic."""
        self.assertEqual(
            set(self.publishers), {f'esp/{command}' for command in ROBOT_BRAIN_COMMANDS})

    def test_each_app_command_publishes_only_on_its_topic(self):
        """Verify each application command publishes one empty message only to its topic."""
        for command in ROBOT_BRAIN_COMMANDS:
            with self.subTest(command=command):
                for publisher in self.publishers.values():
                    publisher.publish.reset_mock()

                getattr(self.app, command)()

                for topic, publisher in self.publishers.items():
                    if topic == f'esp/{command}':
                        publisher.publish.assert_called_once()
                        self.assertIsInstance(publisher.publish.call_args.args[0], FakeEmpty)
                    else:
                        publisher.publish.assert_not_called()

    def test_unknown_command_is_rejected(self):
        """Verify unsupported brain commands raise ValueError."""
        with self.assertRaises(ValueError):
            self.domain.send('explode')


class TestRowDiscovery(unittest.TestCase):
    def setUp(self):
        """Wire discovery services and a view model with workers executed synchronously."""
        self.ros = Mock(spec=RosGateway)
        self.clients = {}

        def create_client(_srv_type, name):
            """Record a mock service client keyed by the final component of its name."""
            self.clients[name.rsplit('/', 1)[-1]] = Mock()
            return self.clients[name.rsplit('/', 1)[-1]]

        self.ros.create_client.side_effect = create_client
        self.domain = RowDiscoveryDomainService(self.ros)
        self.app = RowDiscoveryApplicationService(self.domain)
        self.vm = RunViewModel(Mock(), discovery_app_service=self.app)
        self.thread_patch = patch.object(discovery_module.threading, 'Thread', ImmediateThread)
        self.thread_patch.start()
        self.addCleanup(self.thread_patch.stop)

    def respond(self, name, success=True, message='', error=None):
        """Configure a ready discovery client with a response or request error."""
        client = self.clients[name]
        client.wait_for_service.return_value = True
        client.call_async.return_value = Future(
            SimpleNamespace(success=success, message=message), error)

    def test_talks_to_the_row_discovery_node_services_and_status_topic(self):
        """Verify discovery creates start/stop clients and subscribes to the status topic."""
        self.assertEqual(set(self.clients), {'start_discovery', 'stop_discovery'})
        topic = self.ros.create_subscription.call_args.args[1]
        self.assertEqual(topic, '/row_discovery/status')

    def test_status_feed_updates_the_view_model(self):
        """Verify discovery status messages reach the displayed status."""
        callback = self.ros.create_subscription.call_args.args[2]

        callback(FakeString('row 3 found'))

        self.assertEqual(self.vm.discovery.status, 'row 3 found')

    def test_start_success_marks_discovery_active(self):
        """Verify a successful start sets active state and the default running status."""
        self.respond('start_discovery', success=True)

        self.vm.start_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'running')

    def test_start_failure_keeps_the_service_message(self):
        """Verify a rejected start preserves its diagnostic and leaves discovery inactive."""
        self.respond('start_discovery', success=False, message='no camera')

        self.vm.start_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'no camera')

    def test_start_without_the_node_reports_it_is_not_running(self):
        """Verify an unavailable start service reports an error without sending a request."""
        self.clients['start_discovery'].wait_for_service.return_value = False

        self.vm.start_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: row_discovery_node not running')
        self.clients['start_discovery'].call_async.assert_not_called()

    def test_start_error_is_reported(self):
        """Verify start request exceptions become status errors and leave discovery inactive."""
        self.respond('start_discovery', error=RuntimeError('boom'))

        self.vm.start_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: boom')

    def test_start_shows_starting_before_the_reply(self):
        """Verify a pending start response leaves the UI showing its starting status."""
        self.clients['start_discovery'].wait_for_service.return_value = True
        pending = Mock()
        self.clients['start_discovery'].call_async.return_value = pending

        self.vm.start_discovery()

        self.assertEqual(self.vm.discovery.status, 'starting…')

    def test_stop_without_node_preserves_unknown_discovery_state(self):
        """Verify an unavailable stop service preserves the last active state."""
        self.vm.discovery.active = True
        client = self.clients['stop_discovery']
        client.wait_for_service.return_value = False

        self.vm.stop_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: row_discovery_node not running')
        client.wait_for_service.assert_called_once_with(timeout_sec=2.0)
        client.call_async.assert_not_called()

    def test_missing_discovery_service_does_not_raise(self):
        """Verify absent discovery services are tolerated and report unavailability."""
        vm = RunViewModel(Mock())

        vm.start_discovery()
        vm.stop_discovery()

        self.assertFalse(vm.discovery.active)
        self.assertEqual(vm.discovery.status, 'ERROR: discovery service unavailable')

    def test_stop_success_marks_discovery_inactive(self):
        """Verify a successful stop clears active state and supplies the default stop status."""
        self.vm.discovery.active = True
        self.respond('stop_discovery', success=True)

        self.vm.stop_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'stopped')

    def test_failed_stop_leaves_discovery_marked_active(self):
        """Verify a rejected stop preserves active state and reports uncertainty."""
        self.vm.discovery.active = True
        self.respond('stop_discovery', success=False)

        self.vm.stop_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: stop failed — discovery state unknown')

    def test_stop_error_leaves_discovery_marked_active(self):
        """Verify a stop exception preserves active state and reports the error."""
        self.vm.discovery.active = True
        self.respond('stop_discovery', error=RuntimeError('boom'))

        self.vm.stop_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: boom — discovery state unknown')


if __name__ == '__main__':
    unittest.main()
