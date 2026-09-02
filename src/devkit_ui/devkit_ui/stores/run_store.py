from dataclasses import dataclass


class RunStore:
    @dataclass
    class Joystick:
        pose_lbl: str = 'no odom'

    @dataclass
    class NodeMap:
        map_svg: str = ''
        robot_svg: str = ''

    @dataclass
    class Track:
        prefix: str = ''
        interval: float = 5.0
        row_id: int | None = None
        row_role: str = 'entry'
        running: bool = False
        status: str = ''

    @dataclass
    class DropNode:
        name: str = ''
        row_id: int | None = None
        row_role: str = 'entry'
        row_hint: str = ''
        status: str = ''

    @dataclass
    class Topo:
        current_node: str = '—'
        selected_node: str | None = None
        navigating: bool = False
        nav_status: str = 'idle'
        delete_status: str = ''

    @dataclass
    class Discovery:
        active: bool = False
        status: str = 'idle'

    def __init__(self) -> None:
        self.joystick = self.Joystick()
        self.node_map = self.NodeMap()
        self.track = self.Track()
        self.drop_node = self.DropNode()
        self.topo = self.Topo()
        self.discovery = self.Discovery()
