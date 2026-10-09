"""Plain result types shared by the topology services (no ROS imports)."""
from dataclasses import dataclass


@dataclass(frozen=True)
class SwitchResult:
    """Outcome of asking the topological map manager to switch to a saved map."""
    available: bool = False
    success: bool = False
    message: str = ''


@dataclass(frozen=True)
class PersistResult:
    """How a persisted map change ended up.

    kind is one of:
        'live'           map saved and the map manager switched to it
        'no_srv'         map saved and published; the switch service is not available
        'switch_failed'  map saved and published after the switch failed or timed out
        'skipped'        nothing written (the node already exists on disk)
    """
    kind: str
    detail: str = ''

    def describe(self, success_msg: str) -> str:
        """Return the status text for a change reported as success_msg."""
        if self.kind == 'live':
            return f'{success_msg} — live'
        if self.kind == 'no_srv':
            return f'{success_msg} — live (no srv)'
        if self.kind == 'switch_failed':
            return f'{success_msg} (switch failed: {self.detail})'
        return success_msg
