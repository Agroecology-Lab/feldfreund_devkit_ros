# pylint: disable=duplicate-code
"""
ui_node.py — Sowbot web cockpit on :80
"""

import logging
import os
import threading
from importlib import resources

# The following imports get generated in the Dockerfile, they aren't available to pylint
# pylint: disable=import-error
import rclpy
from nav_msgs.msg import Odometry
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# MISSION: store owns missions.yaml, scheduling, and run recording.
from devkit_ui.application_services.drive_application_service import DriveApplicationService
from devkit_ui.application_services.mission_application_service import MissionApplicationService
from devkit_ui.application_services.navigation_application_service import (
    NavigationApplicationService,
)
from devkit_ui.application_services.plan_application_service import PlanApplicationService
from devkit_ui.application_services.robot_brain_application_service import RobotBrainApplicationService
from devkit_ui.application_services.row_discovery_application_service import (
    RowDiscoveryApplicationService,
)
from devkit_ui.application_services.sim_asset_service import SimAssetService
from devkit_ui.application_services.sim_world_service import SimWorldService
from devkit_ui.application_services.telemetry_application_service import (
    TelemetryApplicationService,
)
from devkit_ui.application_services.topology_application_service import (
    TopologyApplicationService,
)
from devkit_ui.application_services.topology_edit_service import TopologyEditService
from devkit_ui.constants import FIELD27_CENTER, NODE_NAME
from devkit_ui.domain_services.drive_domain_service import DriveDomainService
from devkit_ui.domain_services.mission_domain_service import (
    ERROR,
    INFO,
    NAV_TIMEOUT_SEC,
    OK,
    PROGRESS,
    WARN,
    MissionDomainService,
    MissionError,
    resolve_steps,
)
from devkit_ui.domain_services.navigation_domain_service import NavigationDomainService
from devkit_ui.domain_services.robot_brain_domain_service import RobotBrainDomainService
from devkit_ui.domain_services.row_discovery_domain_service import RowDiscoveryDomainService
from devkit_ui.domain_services.telemetry_domain_service import TelemetryDomainService
from devkit_ui.domain_services.topology_domain_service import TopologyDomainService
from devkit_ui.missions import MissionStore
from devkit_ui.models import TopoDoc

# pylint: enable=import-error
# OBSTACLE: obstacle manager, view model and card helpers
from devkit_ui.obstacles import ObstacleManager
from devkit_ui.pages.delete_dialogs import confirm_delete_row
from devkit_ui.pages.plan.plan_page import PlanPage
from devkit_ui.pages.run.nav_page import NavPage
from devkit_ui.pages.sim.sim_page import SimPage
from devkit_ui.pages.system.system_page import SystemPage
from devkit_ui.process_registry import shutdown_tools
from devkit_ui.qos import SENSOR_QOS
from devkit_ui.ros_gateway import RosGateway
from devkit_ui.view_models.global_view_model import GlobalViewModel
from devkit_ui.view_models.map_authoring_view_model import MapAuthoringViewModel
from devkit_ui.view_models.mission_view_model import MissionViewModel
from devkit_ui.view_models.obstacle_view_model import ObstacleViewModel
from devkit_ui.view_models.plan_view_model import PlanViewModel
from devkit_ui.view_models.robot_brain_view_model import RobotBrainViewModel
from devkit_ui.view_models.run_view_model import RunViewModel
from devkit_ui.view_models.sim_view_model import SimViewModel
from devkit_ui.view_models.telemetry_view_model import TelemetryViewModel
from devkit_ui.view_models.topology_view_model import TopologyViewModel
from devkit_ui.web_auth import UI_PASSWORD_ENV, BasicAuthMiddleware

# ── Import CSS ────────────────────────────────────────────────────────────────

def load_css() -> str:
    """Return the bundled app stylesheet from the package resources."""
    try:
        return resources.files('devkit_ui').joinpath('css/app.css').read_text(encoding='utf-8')
    except FileNotFoundError:
        return ''


_APP_CSS = load_css()

app.on_shutdown(shutdown_tools)

# ── ROS node ──────────────────────────────────────────────────────────────────

MISSION_STATUS_COLOURS = {
    INFO: '#57606a', PROGRESS: '#0969da', WARN: '#9a6700', ERROR: '#cf222e', OK: '#1a7f37'}


class NiceGuiNode:
    """The NiceGUI application. It reaches ROS only through the RosGateway it is given."""

    def __init__(self, ros: RosGateway) -> None:
        """
        Initialize the GUI view models, navigation interfaces, sensor state, and
        mission-planning components.

        In simulation, configure the dedicated fallback GPS source used when no recent real GPS fix
        is available. Register the NiceGUI root page and initialize topology, obstacle, mission,
        safety, and navigation state.
        """
        self._ros = ros
        self._ros.forward_python_logging('devkit_ui')

        # The sim flag is the authoritative signal, plumbed from manage.py's is_sim through
        # devkit.launch.py -> ui.launch.py.
        self._is_sim = self._ros.get_bool_parameter('sim', False)

        # Initialize the domain services using the gateway.
        self._drive_domain_service = DriveDomainService(self._ros)
        self._topo_domain_service = TopologyDomainService(self._ros)
        self._robot_brain_domain_service = RobotBrainDomainService(self._ros)
        self._row_discovery_domain_service = RowDiscoveryDomainService(self._ros)
        self._nav_domain_service = NavigationDomainService(self._ros)
        # FIELD27_CENTER is the datum the sim GPS shim publishes. See FIELD27_CENTER for why a
        # mismatch with the field's real datum is dangerous, not just cosmetic.
        self._telemetry_domain_service = TelemetryDomainService(
            self._ros, is_sim=self._is_sim, fake_gps_datum=FIELD27_CENTER)

        # Set up the application service layers on top of domain services.
        self._drive_app_service = DriveApplicationService(
            self._drive_domain_service)
        self._topo_app_service = TopologyApplicationService(
            self._topo_domain_service
        )
        self._topo_edit_service = TopologyEditService(self._topo_app_service)
        self._robot_brain_app_service = RobotBrainApplicationService(self._robot_brain_domain_service)
        self._row_discovery_app_service = RowDiscoveryApplicationService(
            self._row_discovery_domain_service)
        self._nav_app_service = NavigationApplicationService(self._nav_domain_service)
        self._telemetry_app_service = TelemetryApplicationService(
            self._telemetry_domain_service)
        self._plan_app_service = PlanApplicationService(
            self._topo_app_service, self._topo_edit_service, self._telemetry_app_service,
            is_sim=self._is_sim)

        # Initialize view models
        self._global_vm = GlobalViewModel(self._drive_app_service)
        self._run_vm = RunViewModel(
            self._drive_app_service, self._row_discovery_app_service, self._nav_app_service)
        self._topo_vm = TopologyViewModel(self._topo_app_service)
        self._plan_vm = PlanViewModel(self._plan_app_service, self._topo_vm, self._run_vm)
        self._authoring_vm = MapAuthoringViewModel(
            self._run_vm, self._topo_vm, self._topo_app_service, self._topo_edit_service,
            self._telemetry_app_service, self._ros.create_timer, is_sim=self._is_sim)
        self._telemetry_vm = TelemetryViewModel(self._telemetry_app_service)
        self._robot_brain_vm = RobotBrainViewModel(self._robot_brain_app_service)
        sim_assets = SimAssetService()
        self._sim_vm = SimViewModel(sim_assets, SimWorldService(sim_assets))

        # Initialize with default demo document
        self._topo_app_service.initialize_with_default()

        self._ros.create_subscription(
            String,
            '/current_node',
            lambda m: self._topo_vm.set_current_node(m.data),
            SENSOR_QOS,
        )

        # OBSTACLE: manager owns obstacles.yaml + /obstacles publisher; the view model is what
        # the cards bind to. Start after the telemetry service exists so the getters resolve.
        self._obstacle_mgr = ObstacleManager(
            self._ros, lambda: self.latest_odom, lambda: self.latest_gps)
        self._obstacle_vm = ObstacleViewModel(self._obstacle_mgr, lambda: self.latest_gps)
        self._obstacle_mgr.start()

        # MISSION: store owns missions.yaml, scheduling, and run recording.
        self._mission_store = MissionStore()
        self._mission_store.attach(self._ros)

        self._mission_service = MissionDomainService(self._ros)
        self._mission_vm = MissionViewModel(MissionApplicationService(
            self._mission_store, self._mission_service, self._topo_app_service.get_doc))

        @ui.page('/')
        def page():
            """
            Builds the NiceGUI application content.
            """
            self.content()

    # ── Topology document property (bridge to view model) ──────────────────
    # All existing code references self._topo_doc; this property transparently
    # gets the mirrored doc from the view model, which is kept in sync with
    # the domain service via the callback. Allows gradual refactoring without
    # changing all call sites at once.

    @property
    def _topo_doc(self) -> TopoDoc | None:
        """Get the current topology document (view model's mirror of domain)."""
        return self._topo_vm.topo_doc

    # ── Telemetry (bridge to the telemetry service) ────────────────────────
    # ObstacleManager, MissionStore and the F2C/drop-node code read the latest odometry and GPS
    # fix off the node. These read-only properties keep those call sites unchanged while the
    # state itself lives in the telemetry service.

    @property
    def latest_odom(self) -> Odometry | None:
        """Return the odometry currently driving the UI, or None before any has arrived."""
        return self._telemetry_app_service.latest_odom

    @property
    def latest_gps(self) -> NavSatFix | None:
        """Return the cached GNSS message, including invalid fixes, or None before reception.

        In simulation this may be a fallback fix when no valid real fix is recent.
        """
        return self._telemetry_app_service.latest_gps

    # ── UI shell ──────────────────────────────────────────────────────────────

    def content(self) -> None:
        """Build the Run, Plan, Sim, and System tabs, initially selecting Run."""
        if _APP_CSS:
            ui.add_head_html(f'<style>{_APP_CSS}</style>')
        with ui.tabs().classes('w-full') as tabs:
            tab_run     = ui.tab('Run',     icon='route')
            tab_plan    = ui.tab('Plan',    icon='checklist')
            tab_sim     = ui.tab('Sim',     icon='view_in_ar')
            tab_system  = ui.tab('System',  icon='settings')
        with ui.tab_panels(tabs, value=tab_run).classes('w-full'):
            with ui.tab_panel(tab_run):
                NavPage(
                    global_vm=self._global_vm,
                    run_vm=self._run_vm,
                    topo_vm=self._topo_vm,
                    telemetry_vm=self._telemetry_vm,
                    obstacle_vm=self._obstacle_vm,
                    authoring=self._authoring_vm,
                )
            with ui.tab_panel(tab_plan):
                self._plan_content()
            with ui.tab_panel(tab_sim):
                SimPage(self._sim_vm, self._topo_vm)
            with ui.tab_panel(tab_system):
                SystemPage(
                    telemetry_vm=self._telemetry_vm,
                    robot_brain_vm=self._robot_brain_vm,
                )

    # ── Plan tab ──────────────────────────────────────────────────────────────

    def _plan_content(self) -> None:
        """Build the Plan tab: field boundary, row planner, mission queue, saved missions and obstacles."""
        gps = ((self.latest_gps.latitude, self.latest_gps.longitude)
               if self.latest_gps else None)
        PlanPage(self._plan_vm, self._mission_vm, self._obstacle_vm, gps,
                 self._run_mission, self.cancel_mission, self._confirm_delete_row)

# ─────────────────────────────────────────────────────────────────────────────
# NiceGuiNode mission executor methods
# ─────────────────────────────────────────────────────────────────────────────

    def _run_mission(self, queue: list, status_lbl) -> None:
        """Start executing the queued row mission.

        Parameters:
            queue (list): Tuples containing a row ID, action key, and action parameters.
            status_lbl: UI status label updated with mission progress and outcome.

        Report an empty queue, an active mission, unavailable navigation, or MissionError
        in the label and return; other startup errors propagate.
        Valid missions run on a worker thread.
        """
        def show(level: str, text: str) -> None:
            """Set the status label's text and color for a mission status level."""
            status_lbl.set_text(text)
            status_lbl.style(f'color:{MISSION_STATUS_COLOURS[level]}')

        if not queue:
            show(ERROR, 'ERROR: queue is empty')
            return
        if self._mission_service.running:
            show(ERROR, 'ERROR: mission already running')
            return
        if not self._run_vm.navigation_available:
            show(ERROR, 'ERROR: action client unavailable')
            return
        try:
            steps = resolve_steps(self._topo_doc, queue)
            self._mission_service.start(
                steps, self._send_goal_sync, show,
                lambda: self._global_vm.soft_estop_active)
        except MissionError as exc:
            show(ERROR, f'ERROR: {exc}')

    def _send_goal_sync(self, target: str) -> bool:
        """Navigate to a topology node and block until it ends.

        Call from a worker thread.
        Use a 300-second wait budget after sending the goal, excluding the server wait.
        Mission cancellation or soft e-stop requests cancellation while waiting.
        See NavigationApplicationService.navigate_and_wait for polling and cancellation timing.
        Synchronous navigation errors propagate.

        Returns:
            bool: True if the robot arrived, False if navigation failed, was cancelled, timed
            out, or the action server was unavailable.
        """
        return self._run_vm.navigate_and_wait(
            target, NAV_TIMEOUT_SEC,
            lambda: self._mission_service.cancel_requested or self._global_vm.soft_estop_active)

    def cancel_mission(self) -> None:
        """Signal the executor to stop before its next goal and cancel the active goal."""
        self._mission_service.cancel()
        self._run_vm.cancel_navigation()

    async def _confirm_delete_row(self, row_id: int) -> None:
        """Ask for confirmation, then delete the row from the map."""
        await confirm_delete_row(self._topo_vm, row_id, self._authoring_vm.delete_row)


# ── entrypoints ───────────────────────────────────────────────────────────────

def main() -> None:
    pass


def ros_main() -> None:
    rclpy.init()
    node = Node(NODE_NAME)
    NiceGuiNode(RosGateway(node))  # its callbacks and the registered page keep it alive
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def _protect_dashboard() -> None:
    """Install HTTP and WebSocket Basic authentication when SOWBOT_UI_PASSWORD is nonempty.

    An unset or empty value leaves the dashboard open to any network client.
    """
    password = os.environ.get(UI_PASSWORD_ENV, '')
    if password:
        app.add_middleware(BasicAuthMiddleware, password=password)
        return
    logging.getLogger(NODE_NAME).warning(
        'UI password is not set: the dashboard on port 80, including the joystick, accepts any client on the network')


_protect_dashboard()
app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'
# reload=False is mandatory here. This module is imported (never run as
# __main__) by the `ui_node` console-script entry point, so there's no
# __name__ guard around this call -- every import executes it. NiceGUI's
# default reload=True spins up uvicorn's reload supervisor, which re-imports
# this module in a worker process to load `app`; that re-import re-runs this
# exact line a second time and tries to bind port 80 again while the
# supervisor still holds it -> EADDRINUSE. Hot-reload also has no use case
# in a container that gets rebuilt/restarted on code changes anyway.
ui.run(favicon='🤖', port=80, reload=False)
