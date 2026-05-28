# pylint: disable=duplicate-code
"""
Mission storage and scheduling for the Sowbot webui.

Threading model
---------------
self._missions is an immutable tuple.  All writes replace it wholesale
under self._lock and bump self._version in the same critical section.
Readers grab the tuple reference lock-free (CPython attribute reads are
atomic) and walk a consistent snapshot.

_persist snapshots the list *under* the lock, then writes the file
*outside* the lock so IO never blocks writers.

Scheduling
----------
Each mission carries one optional integer field, repeat_every_hours:

    None  → one-shot: runs once when active, auto-deactivates on success.
    N     → recurring: re-arms N hours after the last successful run.

today_queue() returns (mission_id, row_id, action, action_params) tuples
for every active+due mission.  A mission is due if it has never run,
its last run failed, or its repeat interval has elapsed.

Action registry
---------------
ACTIONS is a module-level dict of ActionDef dataclasses.  Each ActionDef
carries a param_schema (tuple of ParamDef) describing every configurable
implement parameter.

action_ros_msgs(action_key, action_params, enable) returns a list of
(topic, value) pairs the executor should publish:
  enable=True  → Float64 params first (in schema order), then Bool True.
  enable=False → Bool False only.

Topic type is inferred from the value type by the executor (bool → Bool,
float → Float64).  This keeps the store ROS-free and unit-testable.

Adding an implement: one ActionDef entry with its ParamDef list.
No other file needs changing.
"""

from __future__ import annotations

import os
import re
import threading
from dataclasses import dataclass
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
    ros_topic: Optional[str] = None   # Float64 topic; None = UI-only / not published
    precision: int            = 1     # decimal places in UI spinbox


@dataclass(frozen=True)
class ActionDef:
    """Registry entry for one implement action."""
    key:          str
    label:        str
    icon:         str              # emoji shown in UI chip
    tool_topic:   Optional[str]    # std_msgs/Bool enable/disable; None = drive-only
    param_schema: tuple[ParamDef, ...] = ()

    @property
    def default_params(self) -> dict[str, float]:
        return {p.key: p.default for p in self.param_schema}

    def resolve_params(self, stored: Optional[dict]) -> dict[str, float]:
        """Merge stored action_params with schema defaults.
        Missing keys → default.  Unknown keys silently dropped."""
        src = stored or {}
        return {p.key: float(src.get(p.key, p.default))
                for p in self.param_schema}

    def validate_params(self, params: dict) -> Optional[str]:
        for p in self.param_schema:
            if p.key not in params:
                continue
            try:
                v = float(params[p.key])
            except (TypeError, ValueError):
                return f'ERROR: {p.key} must be numeric'
            if not (p.min <= v <= p.max):
                return (f'ERROR: {p.key}={v} out of range '
                        f'[{p.min}–{p.max} {p.unit}]')
        return None


# ── Action / implement registry ───────────────────────────────────────────────
#
# speed_mps lives on /tool/speed_override — shared across actions.
# The executor publishes it before engaging the implement enable; nav2
# should honour it via a speed filter node or TwistMux priority lane.
# All implement-specific params use /tool/<implement>/<param> topics.

ACTIONS: dict[str, ActionDef] = {

    'drive': ActionDef(
        key='drive', label='Drive only', icon='🚜',
        tool_topic=None,
        param_schema=(
            ParamDef('speed_mps', 'Speed', 'm/s',
                     default=0.5, min=0.1, max=1.5, step=0.05,
                     ros_topic='/tool/speed_override', precision=2),
        ),
    ),

    'weed': ActionDef(
        key='weed', label='Weed', icon='🌿',
        tool_topic='/tool/weeder/enable',
        param_schema=(
            ParamDef('speed_mps', 'Speed', 'm/s',
                     default=0.3, min=0.05, max=0.8, step=0.05,
                     ros_topic='/tool/speed_override', precision=2),
            ParamDef('blade_depth_mm', 'Blade depth', 'mm',
                     default=30.0, min=5.0, max=80.0, step=5.0,
                     ros_topic='/tool/weeder/blade_depth_mm', precision=0),
        ),
    ),

    'sow': ActionDef(
        key='sow', label='Sow', icon='🌱',
        tool_topic='/tool/seeder/enable',
        param_schema=(
            ParamDef('speed_mps', 'Speed', 'm/s',
                     default=0.4, min=0.1, max=1.0, step=0.05,
                     ros_topic='/tool/speed_override', precision=2),
            ParamDef('seed_rate_per_m', 'Seed rate', 'seeds/m',
                     default=8.0, min=1.0, max=30.0, step=1.0,
                     ros_topic='/tool/seeder/seed_rate', precision=0),
            ParamDef('sow_depth_mm', 'Sow depth', 'mm',
                     default=20.0, min=5.0, max=60.0, step=5.0,
                     ros_topic='/tool/seeder/sow_depth_mm', precision=0),
        ),
    ),

    'harvest': ActionDef(
        key='harvest', label='Harvest', icon='🌾',
        tool_topic='/tool/harvester/enable',
        param_schema=(
            ParamDef('speed_mps', 'Speed', 'm/s',
                     default=0.4, min=0.05, max=0.8, step=0.05,
                     ros_topic='/tool/speed_override', precision=2),
            ParamDef('cutter_rpm', 'Cutter RPM', 'rpm',
                     default=1500.0, min=500.0, max=3000.0, step=100.0,
                     ros_topic='/tool/harvester/cutter_rpm', precision=0),
            ParamDef('cut_height_mm', 'Cut height', 'mm',
                     default=50.0, min=10.0, max=200.0, step=10.0,
                     ros_topic='/tool/harvester/cut_height_mm', precision=0),
        ),
    ),

    'spray': ActionDef(
        key='spray', label='Spray', icon='💧',
        tool_topic='/tool/sprayer/enable',
        param_schema=(
            ParamDef('speed_mps', 'Speed', 'm/s',
                     default=0.5, min=0.1, max=1.2, step=0.05,
                     ros_topic='/tool/speed_override', precision=2),
            ParamDef('flow_rate_ml_per_m2', 'Flow rate', 'ml/m²',
                     default=200.0, min=10.0, max=1000.0, step=10.0,
                     ros_topic='/tool/sprayer/flow_rate', precision=0),
            ParamDef('nozzle_pressure_bar', 'Nozzle pressure', 'bar',
                     default=2.0, min=0.5, max=5.0, step=0.5,
                     ros_topic='/tool/sprayer/nozzle_pressure', precision=1),
        ),
    ),

    'mow': ActionDef(
        key='mow', label='Mow / mulch', icon='✂️',
        tool_topic='/tool/mower/enable',
        param_schema=(
            ParamDef('speed_mps', 'Speed', 'm/s',
                     default=0.5, min=0.1, max=1.0, step=0.05,
                     ros_topic='/tool/speed_override', precision=2),
            ParamDef('blade_rpm', 'Blade RPM', 'rpm',
                     default=2000.0, min=500.0, max=4000.0, step=100.0,
                     ros_topic='/tool/mower/blade_rpm', precision=0),
            ParamDef('deck_height_mm', 'Deck height', 'mm',
                     default=80.0, min=20.0, max=200.0, step=10.0,
                     ros_topic='/tool/mower/deck_height_mm', precision=0),
        ),
    ),
}


# ── ROS message factory ───────────────────────────────────────────────────────

def action_ros_msgs(action_key: str,
                    action_params: Optional[dict],
                    enable: bool) -> list[tuple[str, Any]]:
    """Return (topic, value) pairs for the executor to publish.

    enable=True  → Float64 param topics first (schema order),
                   then Bool True on tool_topic (if set).
    enable=False → Bool False on tool_topic only.

    The executor infers message type from the value:
        bool  → std_msgs/Bool
        float → std_msgs/Float64
    """
    action = ACTIONS.get(action_key)
    if action is None:
        return []
    out: list[tuple[str, Any]] = []
    if enable:
        resolved = action.resolve_params(action_params)
        for p in action.param_schema:
            if p.ros_topic is not None:
                out.append((p.ros_topic, float(resolved[p.key])))
        if action.tool_topic is not None:
            out.append((action.tool_topic, True))
    else:
        if action.tool_topic is not None:
            out.append((action.tool_topic, False))
    return out


# ── Pure helpers ──────────────────────────────────────────────────────────────

def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _now_utc_str() -> str:
    return _now_utc().strftime(_TS_FMT)


def _parse_ts(s: Optional[str]) -> Optional[datetime]:
    if not s:
        return None
    try:
        return datetime.strptime(s, _TS_FMT).replace(tzinfo=timezone.utc)
    except ValueError:
        return None


def _is_due(mission: dict, now: datetime) -> bool:
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


def _clean_name(raw: str) -> str:
    """Uppercase, strip non-alphanumeric-underscore, collapse spaces to _."""
    return _NAME_CLEAN.sub('', raw.strip().upper().replace(' ', '_'))


def validate_mission(name: str,
                     rows: list,
                     action: str,
                     repeat_every_hours: Optional[int],
                     action_params: Optional[dict] = None) -> Optional[str]:
    """Return an error string, or None if valid."""
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
        cleaned = _clean_name(name)
        # Must be non-empty after cleaning and match the pattern.
        if not cleaned:
            return f'ERROR: name {name!r} contains no valid characters'
        if not _NAME_RE.match(cleaned):
            return f'ERROR: invalid name {cleaned!r}'
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
        node.missions_version  : int                bumps on every write
        node.mission_status    : str                last-action status string

    Mission record schema:

        id:                  str            allocated (MISSION_N), immutable
        name:                str            operator label; defaults to id
        rows:                list[str]      topo entry-node names, ordered
        action:              str            key in ACTIONS
        action_params:       dict | None    per-action parameter values
        repeat_every_hours:  int | None     None = one-shot
        active:              bool
        created_at:          str            UTC timestamp
        last_run_at:         str | None
        last_run_success:    bool | None
    """

    _MUTABLE = frozenset(
        {'name', 'rows', 'action', 'action_params', 'repeat_every_hours', 'active'})

    def __init__(self, path: str = MISSIONS_FILE) -> None:
        self._path      = path
        self._lock      = threading.Lock()
        self._missions: tuple[dict, ...] = ()
        self._version:  int              = 0
        self._node                       = None

    # ── lifecycle ─────────────────────────────────────────────────────────

    def attach(self, node) -> None:
        self._node = node
        node.missions         = ()
        node.missions_version = 0
        node.mission_status   = ''
        threading.Thread(target=self._load, daemon=True).start()

    # ── public API ────────────────────────────────────────────────────────

    def add(self, *,
            rows: list,
            action: str,
            name: str = '',
            action_params: Optional[dict] = None,
            repeat_every_hours: Optional[int] = None,
            active: bool = True) -> Optional[str]:
        err = validate_mission(name, rows, action, repeat_every_hours, action_params)
        if err:
            self._set_status(err)
            return None

        cleaned = _clean_name(name) if name else ''
        ts = _now_utc_str()
        resolved = ACTIONS[action].resolve_params(action_params)

        with self._lock:
            existing = {m.get('id') for m in self._missions}
            i = 1
            while f'MISSION_{i}' in existing:
                i += 1
            mid = f'MISSION_{i}'
            record = {
                'id':                 mid,
                'name':               cleaned or mid,
                'rows':               list(rows),
                'action':             action,
                'action_params':      resolved,
                'repeat_every_hours': None if repeat_every_hours is None else int(repeat_every_hours),
                'active':             bool(active),
                'created_at':         ts,
                'last_run_at':        None,
                'last_run_success':   None,
            }
            self._missions = self._missions + (record,)
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
            self._missions = tuple(m for m in self._missions if m.get('id') != mid)
            self._version += 1
            self._sync_node()
        self._set_status(f'deleting {mid} — writing…')
        threading.Thread(
            target=self._persist,
            args=(f'deleted {mid} · {len(self._missions)} left',),
            daemon=True).start()
        return True

    def update(self, mid: str, **fields) -> bool:
        bad = set(fields) - self._MUTABLE
        if bad:
            self._set_status(f'ERROR: cannot update {sorted(bad)}')
            return False
        with self._lock:
            target = next((m for m in self._missions if m.get('id') == mid), None)
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
            # Re-clean name
            if 'name' in fields:
                fields = {**fields,
                          'name': _clean_name(fields['name']) or merged['id']}
            # Re-resolve params when action or params change
            new_action = merged.get('action', '')
            if ('action_params' in fields or 'action' in fields) and new_action in ACTIONS:
                fields = {**fields,
                          'action_params': ACTIONS[new_action].resolve_params(
                              merged.get('action_params'))}
            self._missions = tuple(
                ({**m, **fields} if m.get('id') == mid else m)
                for m in self._missions)
            self._version += 1
            self._sync_node()
        self._set_status(f'{mid} updated — writing…')
        threading.Thread(
            target=self._persist, args=(f'{mid} saved',), daemon=True).start()
        return True

    def set_active(self, mid: str, active: bool) -> bool:
        return self.update(mid, active=bool(active))

    def record_run(self, mid: str, success: bool) -> bool:
        ts = _now_utc_str()
        with self._lock:
            target = next((m for m in self._missions if m.get('id') == mid), None)
            if target is None:
                self._set_status(f'ERROR: {mid!r} not found')
                return False
            patch: dict = {'last_run_at': ts, 'last_run_success': bool(success)}
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

    # ── derived views ─────────────────────────────────────────────────────

    def today_queue(self) -> list[tuple[str, str, str, dict]]:
        """Active+due missions as (mission_id, row_node, action, resolved_params)."""
        now = _now_utc()
        out: list[tuple[str, str, str, dict]] = []
        for m in self._missions:
            if not m.get('active') or not _is_due(m, now):
                continue
            action = m.get('action')
            if action not in ACTIONS:
                continue
            resolved = ACTIONS[action].resolve_params(m.get('action_params'))
            for r in m.get('rows', ()):
                out.append((m['id'], r, action, resolved))
        return out

    def next_due_in_hours(self, mid: str) -> Optional[float]:
        """0.0 = due now, >0 = hours until due, None = not applicable."""
        m = next((x for x in self._missions if x.get('id') == mid), None)
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
        return max(0.0, (last_at + interval - _now_utc()).total_seconds() / 3600.0)

    def find(self, mid: str) -> Optional[dict]:
        return next((m for m in self._missions if m.get('id') == mid), None)

    # ── internals ─────────────────────────────────────────────────────────

    def _sync_node(self) -> None:
        """Mirror state onto node.  Always called inside lock."""
        if self._node is None:
            return
        self._node.missions         = self._missions
        self._node.missions_version = self._version

    def _set_status(self, s: str) -> None:
        if self._node is not None:
            self._node.mission_status = s

    # ── persistence ───────────────────────────────────────────────────────

    def _load(self) -> None:
        try:
            if not os.path.exists(self._path):
                return
            with open(self._path) as fh:
                doc = yaml.safe_load(fh) or {}
            raw = doc.get('missions', []) or []

            # Migrate: back-fill action_params on old records.
            # Write the migrated file so we don't re-migrate every restart.
            migrated = []
            needs_write = False
            for m in raw:
                action = m.get('action', '')
                if 'action_params' not in m and action in ACTIONS:
                    m = {**m, 'action_params': ACTIONS[action].default_params}
                    needs_write = True
                migrated.append(m)

            with self._lock:
                self._missions = tuple(migrated)
                self._version += 1
                self._sync_node()

            if self._node is not None:
                self._node.get_logger().info(
                    f'Loaded {len(migrated)} missions from {self._path}')

            if needs_write:
                self._persist(f'migrated {len(migrated)} missions')

        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(f'Failed to load missions: {e}')

    def _persist(self, success_msg: str) -> None:
        """Snapshot under lock, write outside lock. Atomic via tmp+rename."""
        # Grab snapshot under lock, then release before doing any IO.
        with self._lock:
            snapshot = list(self._missions)

        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True)
            tmp = self._path + '.tmp'
            with open(tmp, 'w') as fh:
                yaml.dump({'missions': snapshot}, fh,
                          default_flow_style=False, sort_keys=False,
                          allow_unicode=True)
            os.replace(tmp, self._path)
            self._set_status(success_msg)
            if self._node is not None:
                self._node.get_logger().info(f'Missions saved: {len(snapshot)}')
        except Exception as e:
            self._set_status(f'ERROR: {e}')
            if self._node is not None:
                self._node.get_logger().error(f'persist failed: {e}')
