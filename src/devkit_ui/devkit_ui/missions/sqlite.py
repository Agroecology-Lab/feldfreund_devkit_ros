import json
import sqlite3
from collections.abc import Callable

from devkit_ui.time_utils import now_utc_str

_MUTABLE_COLUMNS = frozenset[str](('name', 'rows', 'action', 'action_params', 'repeat_every_hours', 'active'))

class MissionSqliteStore:
    """Owns the mission list and its SQLite persistence."""

    def __init__(self, path: str, on_write_done: Callable[[str], None] | None = None) -> None:
        self._path     = path
        self._on_write_done = on_write_done or (lambda _: None)
        # We can disable same-thread check because we use locks to synchronize access.
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._ensure_schema()

    @property
    def missions(self) -> tuple[dict, ...]:
        """Fetches the list of missions."""
        cursor = self._conn.cursor()
        cursor.execute("SELECT * FROM missions ORDER BY id")
        result = cursor.fetchall()
        return tuple(self._deserialize_row(row) for row in result)

    # ── lifecycle ─────────────────────────────────────────────────────────

    def attach(self, node) -> None:
        """Attaches the store to a node."""
        # Nothing needed here.

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

        ts = now_utc_str()

        mid = f"MISSION_{self._next_id()}"

        mission = self._serialize_row({
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
        })

        cursor = self._conn.cursor()
        cursor.execute("""
        INSERT INTO missions
            (id, name, rows, action, action_params, repeat_every_hours, active, created_at, last_run_at, last_run_success)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            mission['id'],
            mission['name'],
            mission['rows'],
            mission['action'],
            mission['action_params'],
            mission['repeat_every_hours'],
            mission['active'],
            mission['created_at'],
            mission['last_run_at'],
            mission['last_run_success'],
        ))
        self._conn.commit()
        self._on_write_done(f'{mid} created')
        return mid

    def delete(self, mid: str) -> bool:
        """Deletes a mission."""
        cursor = self._conn.cursor()
        cursor.execute("DELETE FROM missions WHERE id = ?", (self._db_id(mid),))
        self._conn.commit()
        self._on_write_done(f'{mid} deleted')
        return True

    def update(self, mid: str, **fields) -> bool:
        """Updates mutable fields on a mission.

        Allowed fields: name, rows, action, repeat_every_hours, active
        """

        serialized_fields = self._serialize_row(fields)

        set_blocks = []
        values = []

        for key, value in serialized_fields.items():
            if key not in _MUTABLE_COLUMNS:
                continue
            set_blocks.append(f"{key} = ?")
            values.append(value)

        if not set_blocks:
            return True

        cursor = self._conn.cursor()
        cursor.execute(f"""
            UPDATE missions
            SET {', '.join(set_blocks)}
            WHERE id = ?
        """, [*values, self._db_id(mid)])
        self._conn.commit()
        self._on_write_done(f'{mid} updated')
        return True

    def find(self, mid: str) -> dict | None:
        """Finds a mission by id."""
        cursor = self._conn.cursor()
        cursor.execute("SELECT * FROM missions WHERE id = ?", (self._db_id(mid),))
        result = cursor.fetchone()
        if result is None:
            return None
        return self._deserialize_row(result)

    def find_by_name(self, name: str) -> dict | None:
        """Finds a mission by name."""
        cursor = self._conn.cursor()
        cursor.execute("SELECT * FROM missions WHERE name = ?", (name,))
        result = cursor.fetchone()
        if result is None:
            return None
        return self._deserialize_row(result)

    def _ensure_schema(self) -> None:
        """Ensures the schema in the database is up to date."""
        cursor = self._conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS missions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT,
                rows TEXT,
                action TEXT,
                action_params TEXT,
                repeat_every_hours INTEGER,
                active INTEGER,
                created_at TEXT,
                last_run_at TEXT,
                last_run_success INTEGER
            )
        """)
        self._conn.commit()

    def _next_id(self) -> int:
        """Gets the next ID."""
        cursor = self._conn.cursor()
        cursor.execute("SELECT MAX(id) FROM missions")
        result = cursor.fetchone()
        if result is None or result[0] is None:
            return 1
        return int(result[0]) + 1

    def _serialize_row(self, row: dict) -> dict:
        """
        Serialize a mission row to something that can be stored in the database.

        Some implementation notes:
        * The ID is stored as an integer, but we expose it as a MISSION_<id> string.
        * Complex fields are saved as JSON strings.

        """
        serialized = dict(row)
        if 'id' in serialized:
            serialized['id'] = self._db_id(serialized['id'])
        if 'rows' in serialized:
            serialized['rows'] = json.dumps(serialized['rows'] or [])
        if 'action_params' in serialized:
            serialized['action_params'] = json.dumps(serialized['action_params'] or {})
        if 'active' in serialized:
            serialized['active'] = 1 if serialized['active'] else 0
        return serialized

    def _db_id(self, mission_id: str | int) -> int:
        """Converts a mission ID to a database ID."""
        if isinstance(mission_id, int):
            return mission_id
        return int(mission_id.replace('MISSION_', ''))

    def _deserialize_row(self, db_row: sqlite3.Row | dict) -> dict:
        """Deserialize a mission row from what was stored in the database."""
        row = dict(db_row)
        return {
            **row,
            'id': f"MISSION_{row['id']}",
            'rows': json.loads(row.get('rows', '[]')),
            'action_params': json.loads(row.get('action_params', '{}')),
            'active': row.get('active', 1) == 1,
        }
