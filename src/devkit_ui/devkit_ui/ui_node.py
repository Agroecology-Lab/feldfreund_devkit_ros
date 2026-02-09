# pylint: disable=duplicate-code
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from gps_msgs.msg import GPSFix
from nicegui import app, ui, ui_run
from nicegui.events import ClickEventArguments
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, Duration, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty

SAFETY_QOS = QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        liveliness=LivelinessPolicy.AUTOMATIC,
                        liveliness_lease_duration=Duration(seconds=1))


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.esp_enable_publisher = self.create_publisher(Empty, 'esp/enable', 1)
        self.esp_disable_publisher = self.create_publisher(Empty, 'esp/disable', 1)
        self.esp_reset_publisher = self.create_publisher(Empty, 'esp/reset', 1)
        self.esp_restart_publisher = self.create_publisher(Empty, 'esp/restart', 1)
        self.esp_configure_publisher = self.create_publisher(Empty, 'esp/configure', 1)
        self.subscription = self.create_subscription(GPSFix, 'gpsfix', self.store_gps, 1)
        self.battery_subscription = self.create_subscription(BatteryState, 'battery_state', self.store_battery, 1)
        self.bumper_front_top_subscription = self.create_subscription(Bool,
                                                                      'bumper/front_top',
                                                                      self.update_bumper_front_top,
                                                                      SAFETY_QOS)
        self.bumper_front_bottom_subscription = self.create_subscription(Bool,
                                                                         'bumper/front_bottom',
                                                                         self.update_bumper_front_bottom,
                                                                         SAFETY_QOS)
        self.bumper_back_subscription = self.create_subscription(Bool,
                                                                 'bumper/back',
                                                                 self.update_bumper_back,
                                                                 SAFETY_QOS)
        self.estop_publisher = self.create_publisher(Bool, 'estop/soft', SAFETY_QOS)
        self.estop_front_subscription = self.create_subscription(Bool,
                                                                 'estop/front',
                                                                 self.update_estop_front,
                                                                 SAFETY_QOS)
        self.estop_back_subscription = self.create_subscription(Bool,
                                                                'estop/back',
                                                                self.update_estop_back,
                                                                SAFETY_QOS)

        self.bumper_front_top_active = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active = False
        self.soft_estop_active = False
        self.estop_front_active = False
        self.estop_back_active = False
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.latest_gps = None
        self.latest_battery = None

        @ui.page('/')
        def page():
            self.content()

    def content(self) -> None:
        with ui.row().classes('items-stretch w-[48rem] gap-3'):  # Add gap between items
            with ui.card().classes('flex-1 text-center items-center'):  # Use flex-1 for equal width distribution
                ui.label('Control').classes('text-2xl')
                ui.joystick(color='blue', size=50,
                            on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                            on_end=lambda _: self.send_speed(0.0, 0.0))
                ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')

                def update_button_appearance(e: ClickEventArguments) -> None:
                    print(f'update_button_appearance: {e}')
                    assert isinstance(e.sender, ui.button)
                    self.toggle_estop()
                    if self.soft_estop_active:
                        e.sender.props('color=red')
                        e.sender.text = 'STOPPED'
                    else:
                        e.sender.props('color=blue')
                        e.sender.text = 'EMERGENCY STOP'
                ui.button('EMERGENCY STOP', color='blue', on_click=update_button_appearance) \
                    .classes('w-40 min-h-[3rem]')
            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Data').classes('text-2xl')
                ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                slider_props = 'readonly selection-color=transparent'
                ui.slider(min=-1, max=1, step=0.05, value=0) \
                    .props(slider_props) \
                    .bind_value(self, 'linear_velocity')
                ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                ui.slider(min=-1, max=1, step=0.05, value=0) \
                    .props(slider_props) \
                    .bind_value(self, 'angular_velocity')
                ui.label('Battery').classes('text-xs mb-[-1.4em]')
                ui.label().bind_text_from(self, 'latest_battery',
                                          lambda msg: f'{msg.percentage * 100:.1f}% ({msg.voltage:.1f}V)' if msg is not None else 'N/A')
            with ui.card().classes('flex-1 text-center items-center'):
                ui.label('Safety').classes('text-2xl')
                ui.label('Bumpers').classes('text-xs mb-[-1.4em]')
                ui.label('Front Top: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'bumper_front_top_active',
                                    lambda active: 'Front Top: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('Front Bottom: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'bumper_front_bottom_active',
                                    lambda active: 'Front Bottom: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('Back: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'bumper_back_active',
                                    lambda active: 'Back: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('E-Stops').classes('text-xs mb-[-1.4em] mt-4')
                ui.label('E-Stop Front: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'estop_front_active', lambda active: 'Front: ' + ('ACTIVE' if active else 'inactive'))
                ui.label('E-Stop Back: ---') \
                    .classes('text-sm') \
                    .bind_text_from(self, 'estop_back_active', lambda active: 'Back: ' + ('ACTIVE' if active else 'inactive'))
        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('ESP Control').classes('text-2xl')
            with ui.row().classes('gap-4'):
                ui.button('Enable', color='green', on_click=lambda: self.esp_enable_publisher.publish(Empty())).classes('w-24')
                ui.button('Disable', color='red', on_click=lambda: self.esp_disable_publisher.publish(Empty())).classes('w-24')
                ui.button('Reset', color='orange', on_click=lambda: self.esp_reset_publisher.publish(Empty())).classes('w-24')
                ui.button('Restart', color='blue', on_click=lambda: self.esp_restart_publisher.publish(Empty())).classes('w-24')
                ui.button('Configure', color='purple', on_click=lambda: self.esp_configure_publisher.publish(Empty())) \
                    .classes('w-24')
        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('GPS Map').classes('text-2xl')
            leaflet = ui.leaflet(center=(51.98278, 7.43440), zoom=16).classes('w-full h-96')
            marker = leaflet.marker(latlng=leaflet.center)

            def update_gps_ui() -> None:
                """Update the UI with the latest GPS data every 2 seconds."""
                if self.latest_gps is not None:
                    leaflet.set_center((self.latest_gps.latitude, self.latest_gps.longitude))
                    marker.move(self.latest_gps.latitude, self.latest_gps.longitude)
            ui.timer(2.0, update_gps_ui)

    def toggle_estop(self) -> None:
        """Toggle the emergency stop state."""
        self.soft_estop_active = not self.soft_estop_active
        msg = Bool()
        msg.data = self.soft_estop_active
        self.estop_publisher.publish(msg)

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear_velocity = x
        self.angular_velocity = y
        self.cmd_vel_publisher.publish(msg)

    def store_gps(self, msg: GPSFix) -> None:
        """Store the latest GPS message."""
        self.latest_gps = msg

    def store_battery(self, msg: BatteryState) -> None:
        """Store the latest battery state."""
        self.latest_battery = msg

    def update_bumper_front_top(self, msg: Bool) -> None:
        self.bumper_front_top_active = msg.data

    def update_bumper_front_bottom(self, msg: Bool) -> None:
        self.bumper_front_bottom_active = msg.data

    def update_bumper_back(self, msg: Bool) -> None:
        self.bumper_back_active = msg.data

    def update_estop_front(self, msg: Bool) -> None:
        self.estop_front_active = msg.data

    def update_estop_back(self, msg: Bool) -> None:
        self.estop_back_active = msg.data


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=80)
