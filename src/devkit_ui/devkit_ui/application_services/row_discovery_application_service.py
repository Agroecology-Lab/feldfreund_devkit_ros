from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from devkit_ui.domain_services.row_discovery_domain_service import (
        RowDiscoveryDomainService,
    )
    from devkit_ui.topo_results import DiscoveryResult


class RowDiscoveryApplicationService:
    """User-intent commands for row discovery."""

    def __init__(self, discovery_service: RowDiscoveryDomainService) -> None:
        """Store the domain service used to control row discovery."""
        self._discovery = discovery_service

    def register_status_callback(self, callback: Callable[[str], None]) -> None:
        """Register the callback that receives live discovery status strings."""
        self._discovery.set_status_callback(callback)

    def start(self, on_done: Callable[[DiscoveryResult], None]) -> None:
        """Request discovery in a worker thread and report the result through on_done."""
        self._discovery.start(on_done)

    def stop(self, on_done: Callable[[DiscoveryResult], None]) -> None:
        """Request a discovery stop in a worker thread and report through on_done."""
        self._discovery.stop(on_done)
