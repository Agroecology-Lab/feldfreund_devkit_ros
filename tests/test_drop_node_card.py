import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'devkit_ui'))

from devkit_ui.stores.run_store import RunStore


def test_run_store_exposes_drop_node_state() -> None:
    store = RunStore()

    assert store.drop_node.name == ''
    assert store.drop_node.row_id is None
    assert store.drop_node.row_role == 'entry'
    assert store.drop_node.current_node == ''
    assert store.drop_node.row_hint == ''
    assert store.drop_node.status == ''
