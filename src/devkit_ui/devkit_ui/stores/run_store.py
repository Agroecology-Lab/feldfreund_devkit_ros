from nicegui import ui


class RunStore:
    pose_lbl: str = '—'

    map_html: ui.html | None = None
    robot_html: ui.html | None = None

    # Track UI references
    track_row_id: ui.number | None = None
    track_row_role: ui.toggle | None = None
    track_row_hint: ui.label | None = None
    track_start_btn: ui.button | None = None
    track_stop_btn: ui.button | None = None
    track_status_lbl: ui.label | None = None

run_store = RunStore()
