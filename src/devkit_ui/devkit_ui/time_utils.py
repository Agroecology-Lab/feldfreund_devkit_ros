from datetime import UTC, datetime

# ISO 8601 UTC — lexicographically sortable, unambiguous, standard.
_TS_FMT        = '%Y-%m-%dT%H:%M:%SZ'
_TS_FMT_LEGACY = '%d-%m-%Y_%H-%M-%S'   # accepted on read, never written

def now_utc() -> datetime:
    return datetime.now(UTC)

def now_utc_str()-> str:
    return now_utc().strftime(_TS_FMT)

def parse_ts(s: str | None) -> datetime | None:
    """Parse a stored timestamp back to UTC datetime. Returns None on
    missing or malformed input — callers treat that as 'never ran'.

    Accepts both current ISO 8601 format and the legacy dd-mm-yyyy_hh-mm-ss
    format so existing YAML files migrate transparently on next save."""
    if not s:
        return None
    try:
        return datetime.strptime(s, _TS_FMT).replace(tzinfo=UTC)
    except ValueError:
        return datetime.strptime(s, _TS_FMT_LEGACY).replace(tzinfo=UTC)
