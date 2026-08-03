class RunStore:
    pose_lbl: str = '—'

    map_svg: str = ''
    robot_svg: str = ''

    # Track data
    track_prefix: str = ''
    track_interval: float = 5.0
    track_row_id: int | None = None
    track_row_role: str = 'entry'
    track_running: bool = False
    track_status: str = ''

run_store = RunStore()
