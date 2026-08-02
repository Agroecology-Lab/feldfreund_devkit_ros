from nicegui import ui


class RunStore:
    pose_lbl: str = '—'

    map_html: ui.html | None = None
    robot_html: ui.html | None = None

    # Track data
    track_prefix: str = ''
    track_interval: float = 5.0
    track_row_id: int | None = None
    track_row_role: str = 'entry'
    track_running: bool = False
    track_status: str = ''

run_store = RunStore()
