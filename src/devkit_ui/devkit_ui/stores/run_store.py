from nicegui import ui


class RunStore:
    pose_lbl: str = '—'

    map_html: ui.html | None = None
    robot_html: ui.html | None = None


run_store = RunStore()
