import sys

import pytest

import agri_field_to_terrain


def test_main_rejects_oversized_grid_before_reading_surface(tmp_path, monkeypatch):
    polygon_dir = tmp_path / 'polygons_2D'
    polygon_dir.mkdir()
    (polygon_dir / 'field1.xml').write_text(
        '<x>0</x><y>0</y>'
        '<x>3000</x><y>0</y>'
        '<x>3000</x><y>3000</y>'
        '<x>0</x><y>3000</y>')
    surface_dir = tmp_path / 'surfaces_3D'
    surface_dir.mkdir()
    (surface_dir / 'field1.ply').touch()

    monkeypatch.setattr(
        sys,
        'argv',
        [
            'agri_field_to_terrain.py',
            '--dataset', str(tmp_path),
            '--field', '1',
            '--out', str(tmp_path),
            '--grid', '1',
        ],
    )
    monkeypatch.setattr(
        agri_field_to_terrain,
        'read_surface',
        lambda _path: pytest.fail('surface was read before grid size validation'),
    )

    with pytest.raises(SystemExit, match='would create more than 4,000,000 cells'):
        agri_field_to_terrain.main()
