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
    def __init__(self, target, daemon=False):
        self.target = target

    def start(self):
        self.target()


class Future:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class TestRobotBrainServices(unittest.TestCase):
    def setUp(self):
        self.ros = Mock(spec=RosGateway)
        self.publishers = {}

        def create_publisher(msg_type, topic, qos):
            self.assertIs(msg_type, FakeEmpty)
            self.publishers[topic] = Mock()
            return self.publishers[topic]

        self.ros.create_publisher.side_effect = create_publisher
        self.domain = RobotBrainDomainService(self.ros)
        self.app = RobotBrainApplicationService(self.domain)

    def test_one_publisher_per_command_on_its_own_topic(self):
        self.assertEqual(
            set(self.publishers), {f'esp/{command}' for command in ROBOT_BRAIN_COMMANDS})

    def test_each_app_command_publishes_only_on_its_topic(self):
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
        with self.assertRaises(ValueError):
            self.domain.send('explode')


class TestRowDiscovery(unittest.TestCase):
    def setUp(self):
        self.ros = Mock(spec=RosGateway)
        self.clients = {}

        def create_client(srv_type, name):
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
        client = self.clients[name]
        client.wait_for_service.return_value = True
        client.call_async.return_value = Future(
            SimpleNamespace(success=success, message=message), error)

    def test_talks_to_the_row_discovery_node_services_and_status_topic(self):
        self.assertEqual(set(self.clients), {'start_discovery', 'stop_discovery'})
        topic = self.ros.create_subscription.call_args.args[1]
        self.assertEqual(topic, '/row_discovery/status')

    def test_status_feed_updates_the_view_model(self):
        callback = self.ros.create_subscription.call_args.args[2]

        callback(FakeString('row 3 found'))

        self.assertEqual(self.vm.discovery.status, 'row 3 found')

    def test_start_success_marks_discovery_active(self):
        self.respond('start_discovery', success=True)

        self.vm.start_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'running')

    def test_start_failure_keeps_the_service_message(self):
        self.respond('start_discovery', success=False, message='no camera')

        self.vm.start_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'no camera')

    def test_start_without_the_node_reports_it_is_not_running(self):
        self.clients['start_discovery'].wait_for_service.return_value = False

        self.vm.start_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: row_discovery_node not running')
        self.clients['start_discovery'].call_async.assert_not_called()

    def test_start_error_is_reported(self):
        self.respond('start_discovery', error=RuntimeError('boom'))

        self.vm.start_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: boom')

    def test_start_shows_starting_before_the_reply(self):
        self.clients['start_discovery'].wait_for_service.return_value = True
        pending = Mock()
        self.clients['start_discovery'].call_async.return_value = pending

        self.vm.start_discovery()

        self.assertEqual(self.vm.discovery.status, 'starting…')

    def test_stop_success_marks_discovery_inactive(self):
        self.vm.discovery.active = True
        self.respond('stop_discovery', success=True)

        self.vm.stop_discovery()

        self.assertFalse(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'stopped')

    def test_failed_stop_leaves_discovery_marked_active(self):
        self.vm.discovery.active = True
        self.respond('stop_discovery', success=False)

        self.vm.stop_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: stop failed — discovery state unknown')

    def test_stop_error_leaves_discovery_marked_active(self):
        self.vm.discovery.active = True
        self.respond('stop_discovery', error=RuntimeError('boom'))

        self.vm.stop_discovery()

        self.assertTrue(self.vm.discovery.active)
        self.assertEqual(self.vm.discovery.status, 'ERROR: boom — discovery state unknown')


if __name__ == '__main__':
    unittest.main()
