"""Plant model output, reproducibility, and recoverable world-file errors."""

import math
import random
import xml.etree.ElementTree as ET

import pytest

import topo_to_forest3d as t


@pytest.mark.parametrize('wrapper', ['<sdf><world name="w">{}</world></sdf>', '<world name="w">{}</world>'])
def test_placement_replaces_only_plants_and_preserves_other_world_content(tmp_path, wrapper):
    world = tmp_path / 'field.world'
    retained = (
        '<gravity>0 0 -9.8</gravity><model name="robot"/>'
        '<include><uri>model://ground</uri><pose>10 20 2 0 0 0</pose></include>'
        '<include><uri>model://irrigation/pipe</uri><name>pipe</name></include>'
        '<include><name>no_uri</name></include>'
    )
    world.write_text(wrapper.format(retained +
        '<include><uri>model://crop/old</uri><name>old_crop</name></include>'
        '<include><uri>model://weed/old</uri><name>old_weed</name></include>'))
    before = _world(world)
    preserved = [ET.tostring(child) for child in list(before)[:-2]]
    rows = [{'a': (10, 20), 'b': (12, 22), 'path': [(10, 20), (12, 20), (12, 22)]},
            {'a': (-2, -3), 'b': (-2, -1)}]

    count = t.place_plants_along_rows(world, rows, 1, 'maize', 0, jitter=0)

    result = _world(world)
    assert [ET.tostring(child) for child in list(result)[:len(preserved)]] == preserved
    crops = _plants(world, 'crop')
    assert count == len(crops) == 8
    assert [p[:2] for _, _, p in crops] == [
        (10, 20), (11, 20), (12, 20), (12, 21), (12, 22), (-2, -3), (-2, -2), (-2, -1)]
    assert [name for name, _, _ in crops] == [f'crop_{i}' for i in range(8)]
    assert all(uri == 'model://crop/maize' for _, uri, _ in crops)
    assert all(p[2:5] == (0, 0, 0) and 0 <= p[5] <= 2 * math.pi for _, _, p in crops)
    assert _plants(world, 'weed') == []


@pytest.mark.parametrize('percentage,expected', [(0, 0), (10, 0), (50, 3), (100, 6)])
def test_weed_percentage_uses_crop_count_and_truncates_fractional_weeds(tmp_path, percentage, expected):
    world = _empty_world(tmp_path)
    rows = [{'a': (0, 0), 'b': (1, 0)}]
    assert t.place_plants_along_rows(world, rows, 1, 'plant', percentage) == 2
    weeds = _plants(world, 'weed')
    assert len(weeds) == expected
    assert [name for name, _, _ in weeds] == [f'weed_{i}' for i in range(expected)]
    assert all(uri == 'model://weed/weed1' for _, uri, _ in weeds)


@pytest.mark.parametrize('radius,upper_bound', [(0, 0.3), (1.5, 1.5)])
def test_weeds_scatter_within_requested_radius_of_crop_using_installed_variants(tmp_path, radius, upper_bound):
    models = tmp_path / 'models'
    for variant in ('zeta', 'alpha', 'incomplete'):
        directory = models / 'weed' / variant
        directory.mkdir(parents=True)
        if variant != 'incomplete':
            (directory / 'model.sdf').write_text('<sdf/>')
    world = _empty_world(tmp_path)
    rows = [{'a': (10, -20), 'b': (10, -20)}]
    t.place_plants_along_rows(world, rows, 1, 'plant', 100, models_path=models,
                              jitter=0, weed_radius=radius, seed=7)
    weeds = _plants(world, 'weed')
    assert len(weeds) == 3
    assert all(uri in {'model://weed/alpha', 'model://weed/zeta'} for _, uri, _ in weeds)
    for _, _, pose in weeds:
        distance = math.hypot(pose[0] - 10, pose[1] + 20)
        assert 0.2 - 0.0001 <= distance <= upper_bound + 0.0001
        assert pose[2:5] == (0, 0, 0)
        assert 0 <= pose[5] <= 2 * math.pi


def test_model_variants_are_sorted_and_ignore_incomplete_models(tmp_path):
    for variant in ('zeta', 'alpha', 'incomplete'):
        directory = tmp_path / 'weed' / variant
        directory.mkdir(parents=True)
        if variant != 'incomplete':
            (directory / 'model.sdf').touch()
    (tmp_path / 'weed' / 'README').write_text('not a model')
    assert t._model_variants(tmp_path, 'weed', 'weed1') == ['alpha', 'zeta']


@pytest.mark.parametrize('state', ['unspecified', 'missing', 'empty', 'incomplete'])
def test_model_variants_fall_back_when_no_valid_models_exist(tmp_path, state):
    if state in ('empty', 'incomplete'):
        (tmp_path / 'weed').mkdir()
    if state == 'incomplete':
        (tmp_path / 'weed' / 'unfinished').mkdir()
    models = None if state == 'unspecified' else tmp_path
    assert t._model_variants(models, 'weed', 'weed1') == ['weed1']


def test_placement_is_idempotent_and_does_not_change_global_random_state(tmp_path):
    world = _empty_world(tmp_path)
    rows = [{'a': (10, -20), 'b': (14, -20)}]
    state = random.getstate()
    t.place_plants_along_rows(world, rows, 1, 'plant', 100, seed=42, jitter=0.1)
    first = world.read_bytes()
    assert random.getstate() == state
    for i, (_, _, pose) in enumerate(_plants(world, 'crop')):
        assert abs(pose[0] - (10 + i)) <= 0.1001
        assert abs(pose[1] + 20) <= 0.1001

    t.place_plants_along_rows(world, rows, 1, 'plant', 100, seed=42, jitter=0.1)
    assert world.read_bytes() == first
    assert random.getstate() == state

    t.place_plants_along_rows(world, rows, 1, 'plant', 100, seed=43, jitter=0.1)
    assert world.read_bytes() != first
    assert len(_plants(world, 'crop')) == 5
    assert len(_plants(world, 'weed')) == 15


def test_empty_rows_remove_stale_plants_without_creating_weeds(tmp_path):
    world = tmp_path / 'field.world'
    world.write_text('<sdf><world><include><uri>model://ground</uri></include>'
                     '<include><uri>model://crop/old</uri></include>'
                     '<include><uri>model://weed/old</uri></include></world></sdf>')
    assert t.place_plants_along_rows(world, [], 1, 'plant', 100) == 0
    assert [inc.findtext('uri') for inc in _world(world).findall('include')] == ['model://ground']


@pytest.mark.parametrize('contents,warning', [(None, 'not found'), ('<sdf><world>', 'cannot parse')])
def test_invalid_world_is_reported_and_left_untouched(tmp_path, capsys, contents, warning):
    world = tmp_path / 'bad.world'
    if contents is not None:
        world.write_text(contents)
    assert t.place_plants_along_rows(world, [{'a': (0, 0), 'b': (1, 0)}], 1, 'plant', 100) == 0
    assert warning in capsys.readouterr().err
    if contents is None:
        assert not world.exists()
    else:
        assert world.read_text() == contents


def _empty_world(tmp_path):
    world = tmp_path / 'field.world'
    world.write_text('<sdf><world name="field"/></sdf>')
    return world


def _world(path):
    root = ET.parse(path).getroot()
    return root if root.tag == 'world' else root.find('world')


def _plants(world, category):
    return [(inc.findtext('name'), inc.findtext('uri'),
             tuple(float(value) for value in inc.findtext('pose').split()))
            for inc in _world(world).findall('include')
            if (inc.findtext('uri') or '').startswith(f'model://{category}/')]
