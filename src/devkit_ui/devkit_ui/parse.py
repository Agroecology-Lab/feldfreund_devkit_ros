import json
from datetime import UTC, datetime

import yaml

from devkit_ui.models import TopoDoc, TopoNode, TopoProperties


def _topo_from_dict(doc: dict) -> TopoDoc:
    nodes = []
    for entry in doc.get('nodes', []):
        n = entry.get('node', {})
        meta = dict(entry.get('meta', {}))

        name = n['name']
        pos = n['pose']['position']
        edges = []
        for e in n.get('edges', []):
            if isinstance(e, dict) and 'node' in e:
                edges.append(e['node'])
            else:
                edges.append(e)
        nodes.append(TopoNode(
            name=name,
            x=float(pos['x']),
            y=float(pos['y']),
            edges=edges,
            meta=meta,
            properties=TopoProperties(**n.get('properties', {})),
            verts=[],
        ))

    return TopoDoc(
        nodes=nodes,
        name=doc.get('name', 'mixed_test_map'),
        metric_map=doc.get('metric_map', ''),
        pointset=doc.get('pointset', ''),
        meta=doc.get('meta', {}),
        actions=doc.get('actions', {}),
        definitions=doc.get('definitions', {}),
        transformation=doc.get('transformation', {}),
    )

def parse_topo_yaml(filename: str) -> TopoDoc:
    with open(filename, encoding='utf-8') as f:
        return _topo_from_dict(yaml.safe_load(f))

def parse_topo_json(json_str: str) -> TopoDoc:
    return _topo_from_dict(json.loads(json_str))

def dump_topo_yaml(doc: TopoDoc, filename: str) -> None:
    with open(filename, 'w', encoding='utf-8') as f:
        data = doc.to_dict()
        data['meta']['last_updated'] = datetime.now(UTC).strftime('%d-%m-%Y_%H-%M-%S')

        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=True)
