import os
import queue
import threading
from collections.abc import Callable

import yaml

from devkit_ui.time_utils import now_utc_str


def _coerce_record(raw: object, path: str) -> dict | None:
    """Validate and normalise a record loaded from YAML.

    Returns a dict with all expected fields present, or None if the
    record is fatally malformed (missing id, wrong container type).
    Logs issues to stderr rather than raising, so one bad record does not
    take down the whole store on startup.

    Optional fields missing from older records are filled in with safe
    defaults so the rest of the code can always do m.get('field') without
    defensive gymnastics.
    """
    if not isinstance(raw, dict):
        print(f'[missions] WARN {path}: skipping non-dict record: {raw!r}',
              flush=True)
        return None

    mid = raw.get('id')
    if not mid or not isinstance(mid, str):
        print(f'[missions] WARN {path}: skipping record with missing/invalid id',
              flush=True)
        return None

    rows = raw.get('rows')
    if not isinstance(rows, list):
        print(f'[missions] WARN {path}: {mid} — non-list rows reset to []',
              flush=True)
        rows = []

    reh = raw.get('repeat_every_hours')
    try:
        reh = None if reh is None else int(reh)
    except (TypeError, ValueError):
        print(f'[missions] WARN {path}: {mid} — malformed repeat_every_hours '
              f'{reh!r} reset to None', flush=True)
        reh = None

    raw_params = raw.get('action_params')
    return {
        'id':                 mid,
        'name':               str(raw.get('name') or mid),
        'rows':               [str(r) for r in rows],
        'action':             str(raw.get('action') or 'drive'),
        'action_params':      raw_params if isinstance(raw_params, dict) else {},
        'repeat_every_hours': reh,
        'active':             bool(raw.get('active', True)),
        'created_at':         str(raw.get('created_at') or now_utc_str()),
        'last_run_at':        raw.get('last_run_at'),
        'last_run_success':   raw.get('last_run_success'),
    }

class MissionYamlStore:
    """Owns the mission list and its YAML persistence.

    After attach(node), the node exposes:

        node.missions          : tuple[dict, ...]   read-only snapshot
        node.missions_version  : int                bumps on every change
        node.mission_status    : str                last-action status

    Mission record schema:

        id:                  'MISSION_1'   (allocated, immutable)
        name:                str           (operator label; defaults to id)
        rows:                list[str]     (topo entry-node names)
        action:              str           (key in ACTIONS)
        action_params:       dict          (per-action parameter overrides)
        repeat_every_hours:  int | None    (None == one-shot)
        active:              bool
        created_at:          str           (ISO 8601 UTC)
        last_run_at:         str  | None
        last_run_success:    bool | None
    """

    def __init__(self, path: str, on_write_done: Callable[[str], None]) -> None:
        self._path     = path
        self._node     = None
        self._missions: tuple[dict, ...] = ()
        self._on_write_done = on_write_done

        # Single background writer. Carries (snapshot, status_msg) pairs.
        # The writer loop drains the queue on each wakeup and keeps only
        # the latest entry, so rapid back-to-back writes converge in one
        # disk round-trip and the status message is never stale.
        self._write_q: queue.SimpleQueue = queue.SimpleQueue()
        threading.Thread(
            target=self._writer_loop, daemon=True, name='missions-writer'
        ).start()

    @property
    def missions(self) -> tuple[dict, ...]:
        return self._missions

    # ── lifecycle ─────────────────────────────────────────────────────────

    def attach(self, node) -> None:
        try:
            if not os.path.exists(self._path):
                return
            with open(self._path, encoding='utf-8') as fh:
                doc = yaml.safe_load(fh) or {}
            raw_list = doc.get('missions', []) or []

            good, skipped = [], 0
            for raw in raw_list:
                record = _coerce_record(raw, self._path)
                if record is not None:
                    good.append(record)
                else:
                    skipped += 1

            self._missions = tuple(good)

            msg = f'Loaded {len(good)} missions from {self._path}'
            if skipped:
                msg += f' ({skipped} invalid record(s) skipped — see stderr)'
            if node is not None:
                self._node = node
                node.get_logger().info(msg)
        except Exception as e:
            if node is not None:
                node.get_logger().warn(f'Failed to load missions: {e}')

    # ── public API ────────────────────────────────────────────────────────

    def add(self,
            rows: list,
            action: str,
            action_params: dict | None = None,
            name: str = '',
            repeat_every_hours: int | None = None,
            active: bool = True) -> str | None:
        """Add a mission. Returns its allocated id, or None on failure.
        Status carries the reason either way."""

        existing_ids = {m.get('id') for m in self._missions}
        i = 1
        while f'MISSION_{i}' in existing_ids:
            i += 1
        mid = f'MISSION_{i}'

        ts = now_utc_str()

        mission = {
            'id':                 mid,
            'name':               name or mid,
            'rows':               list(rows),
            'action':             action,
            'action_params':      action_params or {},
            'repeat_every_hours': (None if repeat_every_hours is None
                                        else int(repeat_every_hours)),
            'active':             bool(active),
            'created_at':         ts,
            'last_run_at':        None,
            'last_run_success':   None,
        }
        self._missions = (*self._missions, mission)
        snapshot = list(self._missions)

        self._write_q.put((snapshot, f'{mid} saved · {len(snapshot)} total'))

        return mid

    def delete(self, mid: str) -> bool:
        self._missions = tuple(
            m for m in self._missions if m.get('id') != mid)
        snapshot = list(self._missions)

        self._write_q.put((snapshot, f'deleted {mid} · {len(snapshot)} left'))
        return True

    def update(self, mid: str, **fields) -> bool:
        """Replace mutable fields on a mission. Allowed fields:
        name, rows, action, repeat_every_hours, active.
        Immutable metadata (id, created_at, last_run_*) is rejected
        — record_run owns the last_run_* fields."""

        self._missions = tuple(
            ({**m, **fields} if m.get('id') == mid else m)
            for m in self._missions)
        snapshot = list(self._missions)

        self._write_q.put((snapshot, f'{mid} saved'))
        return True

    # ── derived views (lock-free snapshots) ──────────────────────────────

    def find(self, mid: str) -> dict | None:
        """Lock-free single-mission lookup by id. Returns the dict from
        the current snapshot — treat as read-only."""
        for m in self._missions:
            if m.get('id') == mid:
                return m
        return None

    def find_by_name(self, name: str) -> dict | None:
        """Lock-free lookup by operator name. Returns the first match.
        Useful for collision checks before add() (e.g. the UI_RUN record
        the executor creates to anchor repeat intervals)."""
        for m in self._missions:
            if m.get('name') == name:
                return m
        return None

    # ── persistence ──────────────────────────────────────────────────────

    def _writer_loop(self) -> None:
        """Single serialised writer. Blocks on the queue, then drains any
        items that arrived while the previous write was in progress. Only
        the most-recent (snapshot, msg) pair is written — intermediate
        states are safe to skip because each snapshot is complete."""
        while True:
            snapshot, msg = self._write_q.get()      # block until work arrives
            while not self._write_q.empty():         # drain pile-up
                try:
                    snapshot, msg = self._write_q.get_nowait()
                except queue.Empty:
                    break
            self._do_write(snapshot, msg)

    def _do_write(self, snapshot: list, success_msg: str) -> None:
        """Atomic write: tmp file + os.replace. Called only from _writer_loop."""
        try:
            dirpath = os.path.dirname(self._path)
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            tmp = self._path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                yaml.dump({'missions': snapshot}, fh,
                          default_flow_style=False, sort_keys=False,
                          allow_unicode=True)
            os.replace(tmp, self._path)
            self._on_write_done(success_msg)
            if self._node is not None:
                self._node.get_logger().info(
                    f'Missions saved: {len(snapshot)} total')
        except Exception as e:
            self._on_write_done(f'ERROR: {e}')
            if self._node is not None:
                self._node.get_logger().error(f'persist failed: {e}')
