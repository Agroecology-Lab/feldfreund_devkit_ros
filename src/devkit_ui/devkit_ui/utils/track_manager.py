import re
from collections.abc import Callable
from typing import Any

from devkit_ui.stores.run_store import run_store


class TrackManager:
    def __init__(self,
                 get_nodes_cb: Callable[[], list],
                 drop_node_cb: Callable[[str, int | None, str | None], None],
                 patch_node_role_cb: Callable[[str, str], None],
                 create_timer_cb: Callable[[float, Callable], Any]):

        # Callbacks to interact with the main application logic
        self._get_nodes = get_nodes_cb
        self._drop_node = drop_node_cb
        self._patch_node_role = patch_node_role_cb
        self._create_timer = create_timer_cb

        # Encapsulated state (Notice they are no longer polluting your main page)
        self._timer = None
        self._counter = 0
        self._prefix = ''
        self._row_id = None
        self._row_role = None
        self._is_row = False
        self._first = True

    @property
    def is_running(self) -> bool:
        return self._timer is not None

    def start(self, prefix: str, interval: float, row_id: int | None, row_role: str | None) -> None:
        prefix = re.sub(r'[^A-Z0-9_]', '', prefix.strip().upper().replace(' ', '_'))
        if not prefix:
            run_store.track_status = 'ERROR: prefix required'
            return

        if self.is_running:
            run_store.track_status = 'ERROR: already running'
            return

        # Fetch existing nodes using the callback
        existing = [n.name for n in self._get_nodes()
                    if n.name.startswith(prefix + '_') and n.name[len(prefix)+1:].isdigit()]

        self._counter = max((int(n[len(prefix)+1:]) for n in existing), default=0)
        self._prefix = prefix
        self._row_id = row_id
        self._row_role = row_role
        self._is_row = row_id is not None
        self._first = True

        run_store.track_running = True

        self._drop()
        self._timer = self._create_timer(interval, self._drop)

    def _drop(self) -> None:
        self._counter += 1
        node_name = f'{self._prefix}_{self._counter}'

        if self._is_row:
            role = 'entry' if self._first else 'middle'
            self._first = False
        else:
            role = self._row_role

        self._drop_node(node_name, self._row_id, role)
        run_store.track_status = f'recording  {node_name}  (#{self._counter})'

    def stop(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        if self._is_row and self._counter > 0:
            last_name = f'{self._prefix}_{self._counter}'
            self._patch_node_role(last_name, 'exit')
            run_store.track_status = (f'stopped — {last_name} marked exit'
                                      f'  (#{self._counter} nodes)')
        else:
            run_store.track_status = f'stopped at #{self._counter}'

        self._counter = 0
        self._prefix = ''
        self._row_id = None
        self._is_row = False
        self._first = True

        run_store.track_running = False
