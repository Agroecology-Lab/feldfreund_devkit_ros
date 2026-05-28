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

The status string accepts a last-writer-wins race for the same reason
obstacle_status does: it's human-readable progress, brief overlap of
"writing…" with "saved" is harmless.

Scheduling
----------
Each mission carries one optional integer field, repeat_every_hours:

    None  → one-shot: runs once when active, auto-deactivates on success.
    N     → recurring: re-arms N hours after the last successful run.

today_queue() is the only scheduler primitive: it returns
(mission_id, row_id, action, action_params) tuples for every active
mission that is *due right now*.  A mission is due if it has never run,
if its last run failed (retry asap — operator-gated, no automatic loop
because the executor is operator-triggered), or if its repeat interval
has elapsed.  There is no cron, no calendar, no time-of-day.

record_run(mid, success) is the convergence point: both the scheduled
executor and any manual "Run now" path call it. That keeps the repeat
interval anchored on actual robot activity rather than wall-clock ticks
that ignore manual runs.

Action registry
---------------
ACTIONS is a module-level dict of ActionDef dataclasses.  Each action
carries:

    key:          str             unique action name (dict key)
    label:        str             human-readable UI label
    icon:         str             emoji for the UI chip
    tool_topic:   str | None      std_msgs/Bool enable/disable topic
    param_schema: list[ParamDef]  ordered list of configurable parameters

ParamDef describes one parameter:

    key:      str            field name stored in action_params dict
    label:    str            UI label
    unit:     str            display suffix (m, rpm, km/h, ml/m², …)
    ros_topic:str            Float64 topic the executor publishes the value on
               None          no ROS publish (pure metadata / executor logic)
    default:  float          value used when mission has no action_params entry
    min:      float
    max:      float
    step:     float

The executor calls action_ros_msgs(action_key, action_params) to get a
list of (topic, msg) pairs to publish before and after each row.  The
Bool enable/disable on tool_topic is always included when tool_topic is
set.  Any ParamDef with a ros_topic gets a std_msgs/Float64 published
with its resolved value.

Adding a fourth action: add an ActionDef entry and ParamDef list.  No
other change needed anywhere — the UI and executor are data-driven.
"""

from __future__ import annotations

import os
import re
import threading
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Any, Optional

import yaml

# ── Constants ─────────────────────────────────────────────────────────────────

MISSIONS_FILE = '/workspace/maps/missions.yaml'

_NAME_RE    = re.compile(r'^[A-Z0-9_]+$')
_NAME_CLEAN = re.compile(r'[^A-Z0-9_]')

_TS_FMT = '%d-%m-%Y_%H-%M-%S'


# ── Parameter / Action dataclasses ────────────────────────────────────────────

@dataclass(frozen=True)
class ParamDef:
    """Schema for one configurable implement parameter."""
    key:       str
    label:     str
    unit:      str
    default:   float
    min:       float
    max:       float
    step:      float
    ros_topic: Optional[str] = None   # Float64 topic; None = UI-only
    precision: int            = 1     # decimal places shown in UI


@dataclass(frozen=True)
class ActionDef:
    """Registry entry for one implement action."""
    key:          str
    label:        str
    icon:         str              # emoji shown in the UI chip
    tool_topic:   Optional[str]    # std_msgs/Bool enable/disable
    param_schema: tuple[ParamDef, ...] = field(default_factory=tuple)

    @property
    def default_params(self) -> dict[str, float]:
        """Return {param_key: default_value} for all parameters."""
        return {p.key: p.default for p in self.param_schema}

    def resolve_params(self, stored: Optional[dict]) -> dict[str, float]:
        """Merge stored action_params with defaults.  Missing keys fall back
        to defaults; extra or unknown keys are silently dropped."""
        stored = stored or {}
        return {p.key: float(stored.get(p.key, p.default))
                for p in self.param_schema}

    def validate_params(self, params: dict) -> Optional[str]:
        """Return an error string if any param value is out of range."""
        for p in self.param_schema:
            if p.key not in params:
                continue
            try:
                v = float(params[p.key])
            except (TypeError, ValueError):
                return f'ERROR: {p.key} must be numeric'
            if not (p.min <= v <= p.max):
                return (f'ERROR: {p.key} {v} out of range '
                        f'[{p.min}–{p.max}]')
        return None


# ── Action registry ───────────────────────────────────────────────────────────
#
# Each entry is an ActionDef. Add a new implement by appending an ActionDef
# with its ParamDef list here; no other file needs changing.
#
# ros_topic naming convention for implement parameters:
#   /tool/<implement>/<param>   e.g. /tool/seeder/seed_rate
# All are std_msgs/Float64.  The executor publishes them once before
# engaging the tool_topic enable.

ACTIONS: dict[str, ActionDef] = {

    'drive': ActionDef(
        key='drive',
        label='Drive only',
        icon='🚜',
        tool_topic=None,
        param_schema=(
            ParamDef(
                key='speed_mps',
                label='Speed',
                unit='m/s',
                default=0.5,
                min=0.1,
                max=1.5,
                step=0.05,
                ros_topic='/tool/speed_override',
                precision=2,
            ),
        ),
    ),

    'weed': ActionDef(
        key='weed',
        label='Weed',
        icon='🌿',
        tool_topic='/tool/weeder/enable',
        param_schema=(
            ParamDef(
                key='speed_mps',
                label='Speed',
                unit='m/s',
                default=0.3,
                min=0.05,
                max=0.8,
                step=0.05,
                ros_topic='/tool/speed_override',
                precision=2,
            ),
            ParamDef(
                key='blade_depth_mm',
                label='Blade depth',
                unit='mm',
                default=30.0,
                min=5.0,
                max=80.0,
                step=5.0,
                ros_topic='/tool/weeder/blade_depth_mm',
                precision=0,
            ),
        ),
    ),

    'sow': ActionDef(
        key='sow',
        label='Sow',
        icon='🌱',
        tool_topic='/tool/seeder/enable',
        param_schema=(
            ParamDef(
                key='speed_mps',
                label='Speed',
                unit='m/s',
                default=0.4,
                min=0.1,
                max=1.0,
                step=0.05,
                ros_topic='/tool/speed_override',
                precision=2,
            ),
            ParamDef(
                key='seed_rate_per_m',
                label='Seed rate',
                unit='seeds/m',
                default=8.0,
                min=1.0,
                max=30.0,
                step=1.0,
                ros_topic='/tool/seeder/seed_rate',
                precision=0,
            ),
            ParamDef(
                key='sow_depth_mm',
                label='Sow depth',
                unit='mm',
                default=20.0,
                min=5.0,
                max=60.0,
                step=5.0,
                ros_topic='/tool/seeder/sow_depth_mm',
                precision=0,
            ),
        ),
    ),

    'harvest': ActionDef(
        key='harvest',
        label='Harvest',
        icon='🌾',
        tool_topic='/tool/harvester/enable',
        param_schema=(
            ParamDef(
                key='speed_mps',
                label='Speed',
                unit='m/s',
                default=0.4,
                min=0.05,
                max=0.8,
                step=0.05,
                ros_topic='/tool/speed_override',
                precision=2,
            ),
            ParamDef(
                key='cutter_rpm',
                label='Cutter RPM',
                unit='rpm',
                default=1500.0,
                min=500.0,
                max=3000.0,
                step=100.0,
                ros_topic='/tool/harvester/cutter_rpm',
                precision=0,
            ),
            ParamDef(
                key='cut_height_mm',
                label='Cut height',
                unit='mm',
                default=50.0,
                min=10.0,
                max=200.0,
                step=10.0,
                ros_topic='/tool/harvester/cut_height_mm',
                precision=0,
            ),
        ),
    ),

    'spray': ActionDef(
        key='spray',
        label='Spray',
        icon='💧',
        tool_topic='/tool/sprayer/enable',
        param_schema=(
            ParamDef(
                key='speed_mps',
                label='Speed',
                unit='m/s',
                default=0.5,
                min=0.1,
                max=1.2,
                step=0.05,
                ros_topic='/tool/speed_override',
                precision=2,
            ),
            ParamDef(
                key='flow_rate_ml_per_m2',
                label='Flow rate',
                unit='ml/m²',
                default=200.0,
                min=10.0,
                max=1000.0,
                step=10.0,
                ros_topic='/tool/sprayer/flow_rate',
                precision=0,
            ),
            ParamDef(
                key='nozzle_pressure_bar',
                label='Nozzle pressure',
                unit='bar',
                default=2.0,
                min=0.5,
                max=5.0,
                step=0.5,
                ros_topic='/tool/sprayer/nozzle_pressure',
                precision=1,
            ),
        ),
    ),

    'mow': ActionDef(
        key='mow',
        label='Mow / mulch',
        icon='✂️',
        tool_topic='/tool/mower/enable',
        param_schema=(
            ParamDef(
                key='speed_mps',
                label='Speed',
                unit='m/s',
                default=0.5,
                min=0.1,
                max=1.0,
                step=0.05,
                ros_topic='/tool/speed_override',
                precision=2,
            ),
            ParamDef(
                key='blade_rpm',
                label='Blade RPM',
                unit='rpm',
                default=2000.0,
                min=500.0,
                max=4000.0,
                step=100.0,
                ros_topic='/tool/mower/blade_rpm',
                precision=0,
            ),
            ParamDef(
                key='deck_height_mm',
                label='Deck height',
                unit='mm',
                default=80.0,
                min=20.0,
                max=200.0,
                step=10.0,
                ros_topic='/tool/mower/deck_height_mm',
                precision=0,
            ),
        ),
    ),
}


# ── ROS message factory ───────────────────────────────────────────────────────

def action_ros_msgs(action_key: str,
                    action_params: Optional[dict],
                    enable: bool) -> list[tuple[str, Any]]:
    """Return a list of (topic, value) pairs the executor should publish.

    enable=True  → publish Float64 params first, then Bool True on tool_topic.
    enable=False → publish Bool False on tool_topic only (params irrelevant).

    The caller is responsible for creating the actual ROS publisher and
    building the message; this function only resolves the values.  Keeping
    ROS-free makes the store unit-testable without a live ROS env.

    Return value schema:
        [('topic_name', float | bool), ...]
    """
    action = ACTIONS.get(action_key)
    if action is None:
        return []

    out: list[tuple[str, Any]] = []

    if enable:
        resolved = action.resolve_params(action_params)
        for p in action.param_schema:
            if p.ros_topic is not None:
                out.append((p.ros_topic, resolved[p.key]))
        if action.tool_topic is not None:
            out.append((action.tool_topic, True))
    else:
        if action.tool_topic is not None:
            out.append((action.tool_topic, False))

    return out


# ── Pure helpers (module-level, easy to test) ──────────────────────────────────

def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _now_utc_str() -> str:
    return _now_utc().strftime(_TS_FMT)


def _parse_ts(s: Optional[str]) -> Optional[datetime]:
    """Parse a stored timestamp back to UTC datetime. Returns None on
    missing or malformed input — callers treat that as 'never ran'."""
    if not s:
        return None
    try:
        return datetime.strptime(s, _TS_FMT).replace(tzinfo=timezone.utc)
    except ValueError:
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
        return True
    return (now - last_at) >= interval


def validate_mission(name: str,
                     rows: list,
                     action: str,
                     repeat_every_hours: Optional[int],
                     action_params: Optional[dict] = None) -> Optional[str]:
    """Return an error string, or None if OK."""
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
    if name:
        clean = _NAME_CLEAN.sub(
            '', name.strip().upper().replace(' ', '_'))
        if clean and not _NAME_RE.match(clean):
            return f'ERROR: invalid name {clean!r}'
    if action_params:
        err = ACTIONS[action].validate_params(action_params)
        if err:
            return err
    return None


# ── MissionStore ──────────────────────────────────────────────────────────────


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
        action_params:       dict | None   (per-action parameters; None = all defaults)
        repeat_every_hours:  int | None    (None == one-shot)
        active:              bool
        created_at:          str           (dd-mm-yyyy_hh-mm-ss UTC)
        last_run_at:         str  | None
        last_run_success:    bool | None

    Backward-compatibility: records loaded from YAML without action_params
    are treated as action_params=None (all defaults applied at runtime).
    Records with an unknown action key are preserved as-is in the file but
    surfaced with an error in the UI.
    """

    _MUTABLE_FIELDS = frozenset(
        {'name', 'rows', 'action', 'action_params', 'repeat_every_hours', 'active'})

    def __init__(self, path: str = MISSIONS_FILE) -> None:
        self._path     = path
        self._lock     = threading.Lock()
        self._missions: tuple[dict, ...] = ()
        self._version: int               = 0
        self._node                       = None

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
            name: str = '',
            action_params: Optional[dict] = None,
            repeat_every_hours: Optional[int] = None,
            active: bool = True) -> Optional[str]:
        """Add a mission. Returns its allocated id, or None on failure.
        Status carries the reason either way."""
        err = validate_mission(name, rows, action, repeat_every_hours, action_params)
        if err:
            self._set_status(err)
            return None

        clean = _NAME_CLEAN.sub(
            '', (name or '').strip().upper().replace(' ', '_'))
        ts = _now_utc_str()

        # Normalise action_params: resolve against schema defaults so stored
        # records are self-contained even if the schema changes later.
        resolved_params = (
            ACTIONS[action].resolve_params(action_params)
            if action in ACTIONS else None
        )

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
                'action_params':      resolved_params,
                'repeat_every_hours': (None if repeat_every_hours is None
                                       else int(repeat_every_hours)),
                'active':             bool(active),
                'created_at':         ts,
                'last_run_at':        None,
                'last_run_success':   None,
            }
            self._missions = self._missions + (mission,)
            self._version += 1
            self._sync_node()

        self._set_status(f'{mid} created — writing…')
        threading.Thread(
            target=self._persist,
            args=(f'{mid} saved · {len(self._missions)} total',),
            daemon=True).start()
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

        self._set_status(f'deleting {mid} — writing…')
        threading.Thread(
            target=self._persist,
            args=(f'deleted {mid} · {len(self._missions)} left',),
            daemon=True).start()
        return True

    def update(self, mid: str, **fields) -> bool:
        """Replace mutable fields on a mission. Allowed fields:
        name, rows, action, action_params, repeat_every_hours, active.
        Immutable metadata (id, created_at, last_run_*) is rejected
        — record_run owns the last_run_* fields.

        When action_params is supplied, it is re-resolved against the
        target action's schema (after any action change) so the stored
        record is always fully self-contained."""
        bad = set(fields) - self._MUTABLE_FIELDS
        if bad:
            self._set_status(
                f'ERROR: cannot update field(s) {sorted(bad)}')
            return False

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
                merged.get('repeat_every_hours'),
                merged.get('action_params'))
            if err:
                self._set_status(err)
                return False

            # Re-clean name on the way in if it changed.
            if 'name' in fields:
                cleaned = _NAME_CLEAN.sub(
                    '', (fields['name'] or '').strip()
                          .upper().replace(' ', '_'))
                fields = {**fields, 'name': cleaned or merged['id']}

            # Re-resolve action_params whenever action or params change.
            new_action = merged.get('action', '')
            if 'action_params' in fields or 'action' in fields:
                raw_params = merged.get('action_params')
                if new_action in ACTIONS:
                    fields = {
                        **fields,
                        'action_params': ACTIONS[new_action].resolve_params(raw_params),
                    }

            self._missions = tuple(
                ({**m, **fields} if m.get('id') == mid else m)
                for m in self._missions)
            self._version += 1
            self._sync_node()

        self._set_status(f'{mid} updated — writing…')
        threading.Thread(
            target=self._persist,
            args=(f'{mid} saved',),
            daemon=True).start()
        return True

    def set_active(self, mid: str, active: bool) -> bool:
        """Convenience wrapper around update(). The common toggle."""
        return self.update(mid, active=bool(active))

    def record_run(self, mid: str, success: bool) -> bool:
        """Record a run outcome. Called by the executor (and by any
        manual 'Run now' path) so the repeat interval re-arms from the
        same anchor regardless of who triggered it.

        Side effect: one-shot missions (repeat_every_hours is None)
        that complete successfully are auto-deactivated."""
        ts = _now_utc_str()
        with self._lock:
            target = next(
                (m for m in self._missions if m.get('id') == mid), None)
            if target is None:
                self._set_status(f'ERROR: {mid!r} not found')
                return False

            patch = {
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

        threading.Thread(
            target=self._persist,
            args=(f'{mid} run recorded ({"ok" if success else "fail"})',),
            daemon=True).start()
        return True

    # ── derived views (lock-free snapshots) ──────────────────────────────

    def today_queue(self) -> list[tuple[str, str, str, dict]]:
        """Active+due missions expanded into
        (mission_id, row_id, action, action_params) tuples in declaration
        order.  Lock-free; the executor walks this list and dispatches
        each tuple in sequence.

        action_params is always a fully-resolved dict (defaults filled
        in) even if the stored record has action_params=None.
        """
        now = _now_utc()
        snapshot = self._missions
        out: list[tuple[str, str, str, dict]] = []
        for m in snapshot:
            if not m.get('active'):
                continue
            if not _is_due(m, now):
                continue
            action = m.get('action')
            if action not in ACTIONS:
                continue  # corrupt record — surfaced in UI elsewhere
            resolved = ACTIONS[action].resolve_params(m.get('action_params'))
            for r in m.get('rows', ()):
                out.append((m['id'], r, action, resolved))
        return out

    def next_due_in_hours(self, mid: str) -> Optional[float]:
        """For the UI 'next due in Xh' chip. Returns:
            0.0  → due now (or last run failed, or never run)
            >0   → hours until due
            None → not applicable (one-shot completed, or unknown id,
                   or inactive mission)."""
        snapshot = self._missions
        m = next((x for x in snapshot if x.get('id') == mid), None)
        if m is None or not m.get('active'):
            return None
        last_at = _parse_ts(m.get('last_run_at'))
        if last_at is None or not m.get('last_run_success'):
            return 0.0
        hrs = m.get('repeat_every_hours')
        if hrs is None:
            return None
        try:
            interval = timedelta(hours=int(hrs))
        except (TypeError, ValueError):
            return 0.0
        delta_s = (last_at + interval - _now_utc()).total_seconds()
        return max(0.0, delta_s / 3600.0)

    def find(self, mid: str) -> Optional[dict]:
        """Lock-free single-mission lookup. Returns the dict reference
        from the current snapshot (immutable in practice — never mutate)."""
        for m in self._missions:
            if m.get('id') == mid:
                return m
        return None

    # ── internals ─────────────────────────────────────────────────────────

    def _sync_node(self) -> None:
        """Mirror internal state onto the node. Called inside the lock."""
        if self._node is None:
            return
        self._node.missions         = self._missions
        self._node.missions_version = self._version

    def _set_status(self, s: str) -> None:
        """Status setter — last-writer-wins race accepted (see module docs)."""
        if self._node is not None:
            self._node.mission_status = s

    # ── persistence ───────────────────────────────────────────────────────

    def _load(self) -> None:
        try:
            if not os.path.exists(self._path):
                return
            with open(self._path) as fh:
                doc = yaml.safe_load(fh) or {}
            loaded = doc.get('missions', []) or []

            # Back-fill action_params for records that pre-date this change.
            migrated = []
            for m in loaded:
                action = m.get('action', '')
                if 'action_params' not in m and action in ACTIONS:
                    m = {**m, 'action_params': ACTIONS[action].default_params}
                migrated.append(m)

            with self._lock:
                self._missions = tuple(migrated)
                self._version += 1
                self._sync_node()
            if self._node is not None:
                self._node.get_logger().info(
                    f'Loaded {len(migrated)} missions from {self._path}')
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(
                    f'Failed to load missions: {e}')

    def _persist(self, success_msg: str) -> None:
        """Atomic write: tmp file + os.replace. Snapshot under lock,
        write lock-free. No publish step — missions aren't a ROS topic."""
        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True)
            tmp = self._path + '.tmp'
            with self._lock:
                snapshot = list(self._missions)
            with open(tmp, 'w') as fh:
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
                self._node.get_logger().error(
                    f'persist failed: {e}')
