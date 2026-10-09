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
        self._discovery = discovery_service

    def register_status_callback(self, callback: Callable[[str], None]) -> None:
        self._discovery.set_status_callback(callback)

    def start(self, on_done: Callable[[DiscoveryResult], None]) -> None:
        self._discovery.start(on_done)

    def stop(self, on_done: Callable[[DiscoveryResult], None]) -> None:
        self._discovery.stop(on_done)
