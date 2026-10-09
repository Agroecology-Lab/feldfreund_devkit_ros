"""File storage for topological maps (no ROS imports)."""
from __future__ import annotations

import os
from typing import TYPE_CHECKING

from devkit_ui.parse import dump_topo_yaml, parse_topo_yaml

if TYPE_CHECKING:
    from devkit_ui.models import TopoDoc

DEFAULT_MAPS_DIR = '/workspace/maps'


class TopoMapStore:
    """Read and write map files in a single directory."""

    def __init__(self, maps_dir: str = DEFAULT_MAPS_DIR) -> None:
        self._dir = str(maps_dir)

    def path(self, name: str) -> str:
        """Return the file path for a map name."""
        return f'{self._dir}/{name}'

    def exists(self, name: str) -> bool:
        return os.path.exists(self.path(name))

    def load(self, name: str) -> TopoDoc:
        return parse_topo_yaml(self.path(name))

    def save(self, doc: TopoDoc, name: str | None = None) -> None:
        """Write doc to <maps_dir>/<name>, creating the directory if needed."""
        os.makedirs(self._dir, exist_ok=True)
        dump_topo_yaml(doc, self.path(name or doc.name))

    def next_archive_name(self, name: str) -> str:
        """Return <name>_<N> for the first N, starting at 1, that is not taken."""
        i = 1
        while self.exists(f'{name}_{i}'):
            i += 1
        return f'{name}_{i}'
