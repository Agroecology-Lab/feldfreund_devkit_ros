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

    def __init__(self) -> None:
        self.joystick = self.Joystick()
        self.node_map = self.NodeMap()
        self.track = self.Track()
