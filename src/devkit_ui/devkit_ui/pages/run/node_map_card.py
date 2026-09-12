from nicegui import ui

from devkit_ui.utils.topo_renderer import build_robot_svg, build_svg, inject_click_js
from devkit_ui.view_models.run_view_model import RunViewModel
from devkit_ui.view_models.topology_view_model import TopologyViewModel


class NodeMapCard(ui.card):
    def __init__(self, topo_vm: TopologyViewModel,
                 pose_state: RunViewModel.NodeMap):
        """
        Initialize a card displaying the node map and robot marker overlay.

        Parameters:
            topo_vm (TopologyViewModel): Topology state providing the map,
                selected node, and current node.
            pose_state (RunViewModel.NodeMap): State providing the latest
                robot pose in map frame.
        """
        super().__init__()

        self._topo_vm = topo_vm
        self._pose_state = pose_state

        # Change detection keys to avoid unnecessary DOM updates
        self._prev_map_key: tuple | None = None
        self._prev_robot_key: tuple | None = None

        self.classes('flex-1')

        with self:
            ui.label('Node Map').classes('sec-label')

            # Node map + robot marker overlay (same viewBox, so they
            # align). Overlay is pointer-events:none so clicks reach
            # the nodes; it updates on movement without rebuilding
            # the clickable node DOM.
            with ui.element('div').classes('relative w-full'):
                self._map_html = ui.html().classes('w-full')
                self._robot_html = ui.html().classes(
                    'absolute top-0 left-0 w-full'
                ).style('pointer-events:none')

        inject_click_js()
        ui.timer(0.2, self._refresh)

    def _refresh(self) -> None:
        """Rebuild and display the map and robot SVG overlays."""
        doc = self._topo_vm.topo_doc
        nodes = doc.nodes if doc else []
        selected = self._topo_vm.selected_node
        current = self._topo_vm.current_node
        pose = self._pose_state.robot_pose

        # Only rebuild the heavy clickable map when topology, selection, or current node changes
        # Key includes: node count + node names (as tuple for identity) + selection + current
        map_key = (len(nodes), tuple(n.name for n in nodes), selected, current)
        if map_key != self._prev_map_key:
            self._prev_map_key = map_key
            self._map_html.set_content(
                build_svg(
                    doc,
                    selected,
                    current,
                )
            )

        # Only rebuild the robot marker overlay when pose or node bounds change
        # Quantize pose to 0.1m/0.01rad to avoid thrashing on tiny float diffs
        robot_pose_key = (
            None if pose is None
            else (round(pose[0], 1), round(pose[1], 1), round(pose[2], 2))
        )
        robot_key = (len(nodes), robot_pose_key)
        if robot_key != self._prev_robot_key:
            self._prev_robot_key = robot_key
            self._robot_html.set_content(
                build_robot_svg(nodes, pose)
            )
