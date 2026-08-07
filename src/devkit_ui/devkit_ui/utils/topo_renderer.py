import html
import math
from collections.abc import Iterator

from nicegui import ui

from devkit_ui.models import TopoDoc, TopoNode

_SVG_W, _SVG_H = 900, 480
_MARGIN, _NODE_R = 56, 17


def _node_transform(nodes: Iterator[TopoNode]):
    """World->screen transform (tx, ty) + extents (dx, dy) for the node bbox.

    Shared by build_svg and build_robot_svg so both layers map coordinates
    identically.
    """
    xs, ys = [], []
    for node in nodes:
        xs.append(node.x)
        ys.append(node.y)
    dx = (max(xs) - min(xs)) or 1.0
    dy = (max(ys) - min(ys)) or 1.0
    x_min, y_min = min(xs), min(ys)

    def tx(x):
        return _MARGIN + (x - x_min) / dx * (_SVG_W - 2 * _MARGIN)
    def ty(y):
        return _SVG_H - _MARGIN - (y - y_min) / dy * (_SVG_H - 2 * _MARGIN)
    return tx, ty, dx, dy


def build_robot_svg(nodes: Iterator[TopoNode], robot: tuple | None) -> str:
    """Transparent overlay SVG with only the live robot marker (map frame).

    Kept separate from build_svg so robot movement updates this layer alone,
    never replacing the clickable node DOM (which would drop its click handlers).
    """
    nodes = list(nodes)
    empty = (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg"></svg>')
    if not nodes or robot is None:
        return empty
    tx, ty, dx, dy = _node_transform(nodes)
    rx, ry, ryaw = robot
    mx, my = tx(rx), ty(ry)
    al = 0.07 * max(dx, dy)
    ex, ey = tx(rx + al * math.cos(ryaw)), ty(ry + al * math.sin(ryaw))
    return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">'
            f'<line x1="{mx:.1f}" y1="{my:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" '
            f'stroke="#cf222e" stroke-width="3" stroke-linecap="round"/>'
            f'<circle cx="{mx:.1f}" cy="{my:.1f}" r="7" fill="#cf222e" '
            f'stroke="#ffffff" stroke-width="2"/></svg>')


def build_svg(doc: TopoDoc, selected: str | None, current: str | None) -> str:
    if not doc.nodes:
        return (f'<svg width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
                f'style="background:#f6f8fa;border-radius:4px;border:1px solid #d0d7de">'
                f'<text x="{_SVG_W//2}" y="{_SVG_H//2}" text-anchor="middle" '
                f'fill="#8c959f" font-family="Courier New" font-size="13">No map loaded</text>'
                f'</svg>')

    tx, ty, _dx, _dy = _node_transform(doc.nodes)

    parts: list[str] = [f'<rect width="{_SVG_W}" height="{_SVG_H}" fill="#f6f8fa" rx="4"/>']

    drawn: set = set()
    for nd in doc.nodes:
        for tgt in nd.edges:
            tgt_name = tgt.node
            key = tuple(sorted([nd.name, tgt_name]))
            if key in drawn or not doc.has_node(tgt_name):
                continue
            drawn.add(key)

            tgt_node = doc.get_node(tgt_name)

            is_row_edge = (nd.meta.get('row_id') is not None
                        or tgt_node.meta.get('row_id') is not None)
            parts.append(
                f'<line x1="{tx(nd.x):.1f}" y1="{ty(nd.y):.1f}" '
                f'x2="{tx(tgt_node.x):.1f}" y2="{ty(tgt_node.y):.1f}" '
                f'stroke="{"#d8e8fd" if is_row_edge else "#d0d7de"}" '
                f'stroke-width="{"2" if is_row_edge else "1"}" stroke-linecap="round"/>')

    for nd in doc.nodes:
        name = nd.name
        cx, cy = tx(nd.x), ty(nd.y)
        is_cur = name == current
        is_sel = name == selected
        is_row = nd.meta.get('row_id') is not None

        # pylint: disable=multiple-statements
        if is_cur:
            fill, stroke, sw = '#dafbe1', '#1a7f37', 2
        elif is_sel:
            fill, stroke, sw = '#fff8c5', '#9a6700', 2
        elif is_row:
            fill, stroke, sw = '#d8e8fd', '#0969da', 1
        elif nd.meta.get('dropped_by'):
            fill, stroke, sw = '#f6f8fa', '#8c959f', 1
        else:
            fill, stroke, sw = '#ffffff', '#d0d7de', 1
        # pylint: enable=multiple-statements

        label_col = ('#1a7f37' if is_cur else '#9a6700' if is_sel
                    else '#0969da' if is_row else '#57606a')

        if is_cur:
            parts.append(
                f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R+6}" '
                f'fill="none" stroke="#1a7f37" stroke-width="1" opacity="0.4">'
                f'<animate attributeName="r" values="{_NODE_R+6};{_NODE_R+13}" '
                f'dur="2s" repeatCount="indefinite"/>'
                f'<animate attributeName="opacity" values="0.4;0" '
                f'dur="2s" repeatCount="indefinite"/></circle>')

        parts.append(
            f'<circle class="topo-node" data-node="{html.escape(str(name), quote=True)}" '
            f'cx="{cx:.1f}" cy="{cy:.1f}" r="{_NODE_R}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

        if is_row:
            role = str(nd.meta.get('row_role') or '?')
            row_id = str(nd.meta.get('row_id', ''))
            label_text = f"{role[0].upper()}{row_id}"
            escaped_text = html.escape(label_text, quote=True)

            parts.append(
                f'<text x="{cx:.1f}" y="{cy+4:.1f}" text-anchor="middle" fill="#0969da" '
                f'font-family="Courier New" font-size="8" font-weight="700" '
                f'style="pointer-events:none">'
                f'{escaped_text}'
                f'</text>')

        parts.append(
            f'<text x="{cx:.1f}" y="{cy+_NODE_R+12:.1f}" text-anchor="middle" '
            f'fill="{label_col}" font-family="Courier New" font-size="9" '
            f'style="pointer-events:none">{html.escape(str(name), quote=True)}</text>')

    return (f'<svg id="topo-map" width="100%" viewBox="0 0 {_SVG_W} {_SVG_H}" '
            f'xmlns="http://www.w3.org/2000/svg">{"".join(parts)}</svg>')


def inject_click_js() -> None:
    # Event delegation: a single document-level listener (attached once,
    # guarded) that resolves clicks to the nearest .topo-node. Survives
    # map redraws — unlike per-element onclick, which gets wiped every
    # time the SVG is rebuilt (on current-node change), leaving the
    # nodes unclickable.
    ui.run_javascript("""
        if (!window.__topoDelegated) {
            window.__topoDelegated = true;
            document.addEventListener('click', (e) => {
                const el = e.target.closest('.topo-node');
                if (el) emitEvent('topo_node_clicked', {node: el.dataset.node});
            });
        }
    """)
