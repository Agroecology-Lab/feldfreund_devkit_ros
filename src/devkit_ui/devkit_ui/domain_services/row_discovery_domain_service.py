from __future__ import annotations

import threading
from collections.abc import Callable

from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from devkit_ui.ros_gateway import RosGateway
from devkit_ui.topo_results import DiscoveryResult

_STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class RowDiscoveryDomainService:
    """Start and stop row_discovery_node and relay its live status feed.

    The node may not be running (the launch file may predate it), so a missing
    service is reported as a DiscoveryResult rather than raised.
    """

    def __init__(self, ros: RosGateway) -> None:
        self._start_cli = ros.create_client(Trigger, '/row_discovery_node/start_discovery')
        self._stop_cli = ros.create_client(Trigger, '/row_discovery_node/stop_discovery')
        self._status_callback: Callable[[str], None] | None = None
        ros.create_subscription(String, '/row_discovery/status', self._on_status, _STATUS_QOS)

    def set_status_callback(self, callback: Callable[[str], None]) -> None:
        """Register a callback for each status string the discovery node publishes."""
        self._status_callback = callback

    def _on_status(self, msg: String) -> None:
        if self._status_callback is not None:
            self._status_callback(msg.data)

    def start(self, on_done: Callable[[DiscoveryResult], None]) -> None:
        """Ask the node to start discovering rows; report the outcome through on_done."""
        def _work() -> None:
            if not self._start_cli.wait_for_service(timeout_sec=2.0):
                on_done(DiscoveryResult(False, 'ERROR: row_discovery_node not running'))
                return

            def _cb(future) -> None:
                try:
                    res = future.result()
                    on_done(DiscoveryResult(res.success, res.message or (
                        'running' if res.success else 'failed to start')))
                except Exception as exc:  # pylint: disable=broad-exception-caught
                    on_done(DiscoveryResult(False, f'ERROR: {exc}'))

            self._start_cli.call_async(Trigger.Request()).add_done_callback(_cb)

        threading.Thread(target=_work, daemon=True).start()

    def stop(self, on_done: Callable[[DiscoveryResult], None]) -> None:
        """Ask the node to stop discovering rows; report the outcome through on_done."""
        def _work() -> None:
            def _cb(future) -> None:
                try:
                    res = future.result()
                    if res.success:
                        on_done(DiscoveryResult(False, res.message or 'stopped'))
                    else:
                        on_done(DiscoveryResult(None, res.message or (
                            'ERROR: stop failed — discovery state unknown')))
                except Exception as exc:  # pylint: disable=broad-exception-caught
                    on_done(DiscoveryResult(None, f'ERROR: {exc} — discovery state unknown'))

            self._stop_cli.call_async(Trigger.Request()).add_done_callback(_cb)

        threading.Thread(target=_work, daemon=True).start()
