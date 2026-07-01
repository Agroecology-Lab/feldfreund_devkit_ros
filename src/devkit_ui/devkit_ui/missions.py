# pylint: disable=duplicate-code
"""
Mission storage and scheduling for the Sowbot webui.

Owns the mission list, its YAML persistence, and the ACTIONS registry.
Designed to be attached to a NiceGuiNode (see MissionStore.attach) so the
node exposes a read-only snapshot and a version counter that the UI timer
polls to decide when to rebuild. Does not inherit from rclpy.node.Node.

Unlike obstacles, missions are not broadcast on a ROS topic — they exist
to drive the executor in ui_node.py, which dispatches `sowbot_row_follow`
goals and toggles tool topics. The store knows nothing about ROS itself.

Threading model
---------------
self._missions is an immutable tuple. All writes replace it wholesale
under self._lock, and bump self._version in the same critical section.
Readers grab the tuple reference lock-free (CPython attribute reads are
atomic) and walk a consistent snapshot. Same pattern as obstacles.py.

Persistence is serialised through a single background writer thread
(self._write_q). Callers snapshot the current tuple inside the lock and
enqueue (snapshot, status_msg). The writer drains the queue, keeping
only the most-recent snapshot per wakeup, so rapid back-to-back writes
do not pile up threads and the status message is never stale.

Scheduling
----------
Each mission carries one optional integer field, repeat_every_hours:

    None  → one-shot: runs once when active, auto-deactivates on success.
    N     → recurring: re-arms N hours after the last successful run.

today_queue() is the only scheduler primitive: it returns
(mission_id, row_id, action, action_params) tuples for every active mission that is
*due right now*. A mission is due if it has never run, if its last run
failed (retry asap — operator-gated, no automatic loop because the
executor is operator-triggered), or if its repeat interval has elapsed.
There is no cron, no calendar, no time-of-day. If that's ever asked
for, it's a separate scheduler, not this one.

record_run(mid, success) is the convergence point: both the scheduled
executor and any manual "Run now" path call it. That keeps the repeat
interval anchored on actual robot activity rather than wall-clock ticks
that ignore manual runs.

reset(mid) is the re-arm path: clears last_run_at / last_run_success and
re-activates. Use it to re-run a completed one-shot (e.g. crop re-planted)
or to immediately retry a failed mission outside the normal scheduler pass.

Action registry
---------------
ACTIONS is a module-level dict for now. The tool_topic is the
std_msgs/Bool topic the executor publishes True/False on to engage and
disengage the implement; None means the action drives only. The dict
shape is the documented seam — when a third action with parameters
(RPM, depth, blade height) arrives, refactor to a proper class.

Timestamp format
----------------
Timestamps are written as ISO 8601 UTC: 2025-04-01T09:30:00Z. Records
written by earlier versions of this module (dd-mm-yyyy_hh-mm-ss) are
read transparently by _parse_ts and silently migrated to ISO 8601 on the
next save.

next_due_in_hours() sentinels
-----------------------------
Import DUE_NOW and DUE_FAILED rather than comparing against magic floats:

    DUE_NOW    (0.0)   mission is active and has never run
    DUE_FAILED (-1.0)  mission is active and last run failed
    > 0.0              hours until the recurring interval elapses
    None               inactive, completed one-shot, or unknown id
"""

from __future__ import annotations

import os
import queue
import re
import threading
from dataclasses import dataclass
from datetime import datetime, timedelta

import yaml


@dataclass(frozen=True)
class ActionDef:
    """Definition of a mission action (drive, weed, etc.)."""
    label: str
    icon: str
    tool_topic: str | None = None
    param_schema: tuple = ()  # Optional parameter definitions for the action

# ── Constants ────────────────────────────────────────────────────────────────

MISSIONS_FILE = '/workspace/maps/missions.yaml'

_NAME_RE    = re.compile(r'^[A-Z0-9_]+$')
_NAME_CLEAN = re.compile(r'[^A-Z0-9_]')

# ISO 8601 UTC — lexicographically sortable, unambiguous, standard.
_TS_FMT        = '%Y-%m-%dT%H:%M:%SZ'
_TS_FMT_LEGACY = '%d-%m-%Y_%H-%M-%S'   # accepted on read, never written

# Sentinels returned by next_due_in_hours().
DUE_NOW:    float = 0.0
DUE_FAILED: float = -1.0

# Action registry — now uses ActionDef dataclass.
ACTIONS: dict[str, ActionDef] = {
    'drive': ActionDef(label='Drive only', icon='🚜', tool_topic=None),
    'weed':  ActionDef(label='Weed',       icon='🌿', tool_topic='/tool/weeder/enable'),
}


def action_ros_msgs(action_key: str,
                    action_params: dict | None,  # pylint: disable=unused-argument
                    enable: bool) -> list[tuple[str, object]]:
    """Map an action + params to the (topic, value) pairs to publish.

    The mission executor calls this once with enable=True before navigating
    a row (engage the implement) and once with enable=False after (disengage),
    publishing each returned pair. A bool value is published as std_msgs/Bool,
    a numeric value as std_msgs/Float64.

    Currently the only ROS seam on an action is tool_topic — a Bool engage/
    disengage line. 'drive' has tool_topic=None and so yields nothing. Per-
    parameter setpoint topics (RPM, depth, blade height) aren't wired yet:
    param_schema carries no topic field, so action_params is accepted for
    forward-compatibility but unused. When a parameter gains its own topic,
    extend the schema and append (param_topic, float(value)) pairs here while
    enable is True.
    """
    adef = ACTIONS.get(action_key)
    if adef is None:
        return []

    msgs: list[tuple[str, object]] = []
    if adef.tool_topic:
        msgs.append((adef.tool_topic, bool(enable)))
    return msgs


# ── Pure helpers (module-level, easy to test) ────────────────────────────────

def _now_utc() -> datetime:
    return datetime.now(datetime.UTC)


def _now_utc_str() -> str:
    return _now_utc().strftime(_TS_FMT)


def _parse_ts(s: str | None) -> datetime | None:
    """Parse a stored timestamp back to UTC datetime. Returns None on
    missing or malformed input — callers treat that as 'never ran'.

    Accepts both current ISO 8601 format and the legacy dd-mm-yyyy_hh-mm-ss
    format so existing YAML files migrate transparently on next save."""
    if not s:
        return None
    for fmt in (_TS_FMT, _TS_FMT_LEGACY):
        try:
            return datetime.strptime(s, fmt).replace(tzinfo=datetime.UTC)
        except ValueError:
            pass
    return None


def _is_due(mission: dict, now: datetime) -> bool:
    """Whether the mission belongs in today_queue at `now`.

    Due if any of:
      - never run yet (no last_run_at), or
      - last run failed (retry on next executor pass), or
      - repeat_every_hours is set and the interval has elapsed.

    Not due if repeat_every_hours is None and last_run_success is True
    — a successful one-shot. record_run also flips its active flag, so
    in practice such a mission won't reach this check, but the guard
    keeps the logic correct if a one-shot is somehow active+completed.
    """
    last_at = _parse_ts(mission.get('last_run_at'))
    if last_at is None:
        return True
    if not mission.get('last_run_success'):
        return True
    hrs = mission.get('repeat_every_hours')
    if hrs is None:
        return False
    try:
        interval = timedelta(hours=int(hrs))
    except (TypeError, ValueError):
        # Malformed interval — conservatively due so the UI surfaces it.
        return True
    return (now - last_at) >= interval


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
        'created_at':         str(raw.get('created_at') or _now_utc_str()),
        'last_run_at':        raw.get('last_run_at'),
        'last_run_success':   raw.get('last_run_success'),
    }


def validate_mission(name: str,
                     rows: list,
                     action: str,
                     repeat_every_hours: int | None) -> str | None:
    """Return an error string, or None if OK.

    Expects name to already be cleaned (uppercase, only [A-Z0-9_]).
    MissionStore.add() and .update() clean before calling; external
    callers should do the same or pass name='' to use the id default.
    """
    if not rows:
        return 'ERROR: no rows selected'
    if action not in ACTIONS:
        return f'ERROR: unknown action {action!r}'
    if repeat_every_hours is not None:
        try:
            n = int(repeat_every_hours)
        except (TypeError, ValueError):
            return 'ERROR: repeat_every_hours must be an integer'
        if n <= 0:
            return 'ERROR: repeat_every_hours must be > 0'
    if name and not _NAME_RE.match(name):
        return f'ERROR: invalid name {name!r}'
    return None


# ── MissionStore ─────────────────────────────────────────────────────────────


class MissionStore:
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

    _MUTABLE_FIELDS = frozenset(
        {'name', 'rows', 'action', 'action_params', 'repeat_every_hours', 'active'})

    def __init__(self, path: str = MISSIONS_FILE) -> None:
        self._path     = path
        self._lock     = threading.Lock()
        self._missions: tuple[dict, ...] = ()
        self._version: int               = 0
        self._node                       = None

        # Single background writer. Carries (snapshot, status_msg) pairs.
        # The writer loop drains the queue on each wakeup and keeps only
        # the latest entry, so rapid back-to-back writes converge in one
        # disk round-trip and the status message is never stale.
        self._write_q: queue.SimpleQueue = queue.SimpleQueue()
        threading.Thread(
            target=self._writer_loop, daemon=True, name='missions-writer'
        ).start()

    # ── lifecycle ─────────────────────────────────────────────────────────

    def attach(self, node) -> None:
        """Wire into a NiceGuiNode. Kicks off a background load."""
        self._node = node
        node.missions          = ()
        node.missions_version  = 0
        node.mission_status    = ''
        threading.Thread(target=self._load, daemon=True).start()

    # ── public API ────────────────────────────────────────────────────────

    def add(self, *,
            rows: list,
            action: str,
            action_params: dict | None = None,
            name: str = '',
            repeat_every_hours: int | None = None,
            active: bool = True) -> str | None:
        """Add a mission. Returns its allocated id, or None on failure.
        Status carries the reason either way."""
        # Clean name before validation so validate_mission sees the final form.
        clean = _NAME_CLEAN.sub(
            '', (name or '').strip().upper().replace(' ', '_'))

        err = validate_mission(clean, rows, action, repeat_every_hours)
        if err:
            self._set_status(err)
            return None

        ts = _now_utc_str()

        with self._lock:
            existing_ids = {m.get('id') for m in self._missions}
            i = 1
            while f'MISSION_{i}' in existing_ids:
                i += 1
            mid = f'MISSION_{i}'

            mission = {
                'id':                 mid,
                'name':               clean or mid,
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
            self._version += 1
            self._sync_node()
            snapshot = list(self._missions)

        self._set_status(f'{mid} created — writing…')
        self._write_q.put((snapshot, f'{mid} saved · {len(snapshot)} total'))
        return mid

    def delete(self, mid: str) -> bool:
        with self._lock:
            if not any(m.get('id') == mid for m in self._missions):
                self._set_status(f'ERROR: {mid!r} not found')
                return False
            self._missions = tuple(
                m for m in self._missions if m.get('id') != mid)
            self._version += 1
            self._sync_node()
            snapshot = list(self._missions)

        self._set_status(f'deleting {mid} — writing…')
        self._write_q.put((snapshot, f'deleted {mid} · {len(snapshot)} left'))
        return True

    def update(self, mid: str, **fields) -> bool:
        """Replace mutable fields on a mission. Allowed fields:
        name, rows, action, repeat_every_hours, active.
        Immutable metadata (id, created_at, last_run_*) is rejected
        — record_run owns the last_run_* fields."""
        bad = set(fields) - self._MUTABLE_FIELDS
        if bad:
            self._set_status(f'ERROR: cannot update field(s) {sorted(bad)}')
            return False

        # Clean name before merging so validate_mission sees the final form.
        if 'name' in fields:
            fields = {**fields, 'name': _NAME_CLEAN.sub(
                '', (fields['name'] or '').strip().upper().replace(' ', '_'))}

        with self._lock:
            target = next(
                (m for m in self._missions if m.get('id') == mid), None)
            if target is None:
                self._set_status(f'ERROR: {mid!r} not found')
                return False

            merged = {**target, **fields}
            err = validate_mission(
                merged.get('name', ''),
                merged.get('rows', []),
                merged.get('action', ''),
                merged.get('repeat_every_hours'))
            if err:
                self._set_status(err)
                return False

            # Empty name after cleaning → fall back to id.
            if 'name' in fields and not fields['name']:
                fields = {**fields, 'name': mid}

            self._missions = tuple(
                ({**m, **fields} if m.get('id') == mid else m)
                for m in self._missions)
            self._version += 1
            self._sync_node()
            snapshot = list(self._missions)

        self._set_status(f'{mid} updated — writing…')
        self._write_q.put((snapshot, f'{mid} saved'))
        return True

    def set_active(self, mid: str, active: bool) -> bool:
        """Convenience wrapper around update(). The common toggle."""
        return self.update(mid, active=bool(active))

    def reset(self, mid: str) -> bool:
        """Re-arm a mission: clear run history and re-activate.

        The canonical path for:
          - a completed one-shot that needs to run again (e.g. crop re-planted),
          - a failed mission the operator wants to retry without waiting for
            the next scheduled executor pass.

        Does not touch repeat_every_hours, rows, or action.
        """
        with self._lock:
            if not any(m.get('id') == mid for m in self._missions):
                self._set_status(f'ERROR: {mid!r} not found')
                return False

            patch = {
                'last_run_at':      None,
                'last_run_success': None,
                'active':           True,
            }
            self._missions = tuple(
                ({**m, **patch} if m.get('id') == mid else m)
                for m in self._missions)
            self._version += 1
            self._sync_node()
            snapshot = list(self._missions)

        self._set_status(f'{mid} reset — writing…')
        self._write_q.put((snapshot, f'{mid} reset'))
        return True

    def record_run(self, mid: str, success: bool) -> bool:
        """Record a run outcome. Called by the executor (and any 'Run now'
        path) so the repeat interval re-arms from actual robot activity
        regardless of who triggered the run.

        Side effect: one-shot missions (repeat_every_hours is None) that
        complete successfully are auto-deactivated.
        """
        ts = _now_utc_str()
        with self._lock:
            target = next(
                (m for m in self._missions if m.get('id') == mid), None)
            if target is None:
                self._set_status(f'ERROR: {mid!r} not found')
                return False

            patch: dict = {
                'last_run_at':      ts,
                'last_run_success': bool(success),
            }
            if target.get('repeat_every_hours') is None and success:
                patch['active'] = False

            self._missions = tuple(
                ({**m, **patch} if m.get('id') == mid else m)
                for m in self._missions)
            self._version += 1
            self._sync_node()
            snapshot = list(self._missions)

        self._write_q.put(
            (snapshot, f'{mid} run recorded ({"ok" if success else "fail"})'))
        return True

    # ── derived views (lock-free snapshots) ──────────────────────────────

    def today_queue(self) -> list[tuple[str, str, str, dict]]:
        """Active+due missions expanded into (mission_id, row_id, action,
        action_params) tuples in declaration order. Lock-free; the executor
        walks this list and dispatches each tuple in sequence."""
        now = _now_utc()
        out: list[tuple[str, str, str, dict]] = []
        for m in self._missions:
            if not m.get('active'):
                continue
            if not _is_due(m, now):
                continue
            action = m.get('action')
            if action not in ACTIONS:
                continue  # corrupt record — surfaced in UI via _coerce_record
            params = m.get('action_params') or {}
            for r in m.get('rows', ()):
                out.append((m['id'], r, action, params))
        return out

    def next_due_in_hours(self, mid: str) -> float | None:
        """For the UI chip. Returns one of:

            DUE_NOW    (0.0)   active, never run yet
            DUE_FAILED (-1.0)  active, last run failed — retry sentinel
            > 0.0              hours until the recurring interval elapses
            None               inactive, completed one-shot, or unknown id

        Import DUE_NOW / DUE_FAILED from this module rather than
        comparing against magic floats.  UI rendering pattern:

            h = store.next_due_in_hours(mid)
            if h is None:          label = 'done'
            elif h == DUE_FAILED:  label = 'retry'
            elif h == DUE_NOW:     label = 'due now'
            else:                  label = f'in {h:.1f}h'
        """
        m = next((x for x in self._missions if x.get('id') == mid), None)
        if m is None or not m.get('active'):
            return None
        last_at = _parse_ts(m.get('last_run_at'))
        if last_at is None:
            return DUE_NOW
        if not m.get('last_run_success'):
            return DUE_FAILED
        hrs = m.get('repeat_every_hours')
        if hrs is None:
            return None   # successful one-shot, now inactive in practice
        try:
            interval = timedelta(hours=int(hrs))
        except (TypeError, ValueError):
            return DUE_NOW
        delta_s = (last_at + interval - _now_utc()).total_seconds()
        return max(DUE_NOW, delta_s / 3600.0)

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

    # ── internals ────────────────────────────────────────────────────────

    def _sync_node(self) -> None:
        """Mirror internal state onto the node. Always called inside the lock."""
        if self._node is None:
            return
        self._node.missions         = self._missions
        self._node.missions_version = self._version

    def _set_status(self, s: str) -> None:
        """Status setter — last-writer-wins race accepted (see module docs)."""
        if self._node is not None:
            self._node.mission_status = s

    # ── persistence ──────────────────────────────────────────────────────

    def _load(self) -> None:
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

            with self._lock:
                self._missions = tuple(good)
                self._version += 1
                self._sync_node()

            msg = f'Loaded {len(good)} missions from {self._path}'
            if skipped:
                msg += f' ({skipped} invalid record(s) skipped — see stderr)'
            if self._node is not None:
                self._node.get_logger().info(msg)
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(f'Failed to load missions: {e}')

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
            self._set_status(success_msg)
            if self._node is not None:
                self._node.get_logger().info(
                    f'Missions saved: {len(snapshot)} total')
        except Exception as e:
            self._set_status(f'ERROR: {e}')
            if self._node is not None:
                self._node.get_logger().error(f'persist failed: {e}')
