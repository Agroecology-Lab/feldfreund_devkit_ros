import contextlib
import io
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock

import yaml

from devkit_ui.missions.yaml import MissionYamlStore, _coerce_record  # pylint: disable=protected-access


def coerce(raw: object) -> dict | None:
    """Run _coerce_record with its stdout warnings suppressed."""
    with contextlib.redirect_stdout(io.StringIO()):
        return _coerce_record(raw, 'missions.yaml')


class TestCoerceRecord(unittest.TestCase):
    def test_non_dict_record_is_skipped(self) -> None:
        self.assertIsNone(coerce(['MISSION_1']))

    def test_missing_or_non_string_id_is_skipped(self) -> None:
        self.assertIsNone(coerce({'name': 'no id'}))
        self.assertIsNone(coerce({'id': 7}))

    def test_minimal_record_gets_safe_defaults(self) -> None:
        record = coerce({'id': 'MISSION_1'})

        assert record is not None
        self.assertEqual(record['name'], 'MISSION_1')
        self.assertEqual(record['rows'], [])
        self.assertEqual(record['action'], 'drive')
        self.assertEqual(record['action_params'], {})
        self.assertIsNone(record['repeat_every_hours'])
        self.assertTrue(record['active'])
        self.assertTrue(record['created_at'])
        self.assertIsNone(record['last_run_at'])
        self.assertIsNone(record['last_run_success'])

    def test_full_record_is_preserved(self) -> None:
        record = coerce({
            'id': 'MISSION_2',
            'name': 'NORTH',
            'rows': ['R1', 'R2'],
            'action': 'weed',
            'action_params': {'rpm': 3},
            'repeat_every_hours': '12',
            'active': False,
            'created_at': '2025-04-01T09:30:00Z',
            'last_run_at': '2025-04-02T09:30:00Z',
            'last_run_success': True,
        })

        self.assertEqual(record, {
            'id': 'MISSION_2',
            'name': 'NORTH',
            'rows': ['R1', 'R2'],
            'action': 'weed',
            'action_params': {'rpm': 3},
            'repeat_every_hours': 12,
            'active': False,
            'created_at': '2025-04-01T09:30:00Z',
            'last_run_at': '2025-04-02T09:30:00Z',
            'last_run_success': True,
        })

    def test_malformed_fields_are_reset_instead_of_dropping_the_record(self) -> None:
        record = coerce({
            'id': 'MISSION_3',
            'rows': 'not-a-list',
            'repeat_every_hours': 'often',
            'action_params': 'not-a-dict',
        })

        assert record is not None
        self.assertEqual(record['rows'], [])
        self.assertIsNone(record['repeat_every_hours'])
        self.assertEqual(record['action_params'], {})


class NoDiskYamlStore(MissionYamlStore):
    """MissionYamlStore whose background writer never touches the disk.

    Every mutation queues a write to a daemon thread that outlives the test. Overriding the
    writer, rather than patching it for the duration of a test, means a late write can never
    recreate a temporary directory that has already been cleaned up.
    """

    def _do_write(self, snapshot: list, success_msg: str) -> None:  # pylint: disable=unused-argument
        return


class TestMissionYamlStore(unittest.TestCase):
    """In-memory behaviour; nothing is written to disk."""

    def setUp(self) -> None:
        tmp = self.enterContext(tempfile.TemporaryDirectory())  # pylint: disable=consider-using-with
        self.path = Path(tmp) / 'missions.yaml'
        self.node = Mock()
        self.store = NoDiskYamlStore(str(self.path), on_write_done=lambda _: None)

    def test_attach_without_file_leaves_store_empty(self) -> None:
        self.store.attach(self.node)

        self.assertEqual(self.store.missions, ())

    def test_attach_loads_valid_records_and_skips_invalid_ones(self) -> None:
        self.path.write_text(yaml.safe_dump({'missions': [
            {'id': 'MISSION_1', 'rows': ['R1'], 'action': 'drive'},
            'garbage',
            {'name': 'missing id'},
        ]}), encoding='utf-8')

        with contextlib.redirect_stdout(io.StringIO()):
            self.store.attach(self.node)

        self.assertEqual([m['id'] for m in self.store.missions], ['MISSION_1'])
        message = self.node.get_logger().info.call_args.args[0]
        self.assertIn('Loaded 1 missions', message)
        self.assertIn('2 invalid record(s) skipped', message)

    def test_attach_with_unreadable_yaml_logs_a_warning(self) -> None:
        self.path.write_text('missions: [unclosed', encoding='utf-8')

        self.store.attach(self.node)

        self.assertEqual(self.store.missions, ())
        self.assertIn('Failed to load missions', self.node.get_logger().warn.call_args.args[0])

    def test_add_allocates_the_lowest_free_id(self) -> None:
        first = self.store.add(rows=['R1'], action='drive')
        second = self.store.add(rows=['R2'], action='weed', name='SECOND', repeat_every_hours=6)
        self.assertTrue(self.store.delete('MISSION_1'))
        third = self.store.add(rows=['R3'], action='drive')

        self.assertEqual((first, second, third), ('MISSION_1', 'MISSION_2', 'MISSION_1'))

    def test_find_and_find_by_name(self) -> None:
        self.store.add(rows=['R1'], action='drive', name='FIRST')
        self.store.add(rows=['R2'], action='weed', name='SECOND')

        self.assertEqual(self.store.find('MISSION_2'), self.store.find_by_name('SECOND'))
        self.assertIsNone(self.store.find('MISSION_99'))
        self.assertIsNone(self.store.find_by_name('NOPE'))

    def test_update_replaces_fields_on_the_matching_mission_only(self) -> None:
        self.store.add(rows=['R1'], action='drive')
        self.store.add(rows=['R2'], action='drive')

        self.assertTrue(self.store.update('MISSION_2', name='RENAMED', active=False))

        renamed = self.store.find('MISSION_2')
        untouched = self.store.find('MISSION_1')
        assert renamed is not None and untouched is not None
        self.assertEqual(renamed['name'], 'RENAMED')
        self.assertFalse(renamed['active'])
        self.assertEqual(untouched['name'], 'MISSION_1')
        self.assertTrue(untouched['active'])


class TestMissionYamlStorePersistence(unittest.TestCase):
    """Real writes. Each test triggers exactly one write and waits for it."""

    def setUp(self) -> None:
        tmp = self.enterContext(tempfile.TemporaryDirectory())  # pylint: disable=consider-using-with
        self.dir = Path(tmp)
        self.path = self.dir / 'missions.yaml'
        self.written = threading.Event()
        self.statuses: list[str] = []
        self.node = Mock()

    def on_write_done(self, message: str) -> None:
        self.statuses.append(message)
        self.written.set()

    def wait_for_write(self) -> None:
        self.assertTrue(self.written.wait(timeout=5), 'writer thread did not finish')

    def test_added_mission_is_written_to_disk_and_reloads_in_a_new_store(self) -> None:
        store = MissionYamlStore(str(self.path), on_write_done=self.on_write_done)
        store.add(rows=['R1', 'R2'], action='weed', name='PERSISTED', repeat_every_hours=4)
        self.wait_for_write()

        reloaded = MissionYamlStore(str(self.path), on_write_done=self.on_write_done)
        reloaded.attach(self.node)

        mission = reloaded.find_by_name('PERSISTED')
        assert mission is not None
        self.assertEqual(mission['rows'], ['R1', 'R2'])
        self.assertEqual(mission['action'], 'weed')
        self.assertEqual(mission['repeat_every_hours'], 4)
        self.assertEqual(self.statuses, ['MISSION_1 saved · 1 total'])

    def test_write_is_atomic_and_leaves_no_temporary_file(self) -> None:
        store = MissionYamlStore(str(self.path), on_write_done=self.on_write_done)
        store.add(rows=['R1'], action='drive')
        self.wait_for_write()

        self.assertEqual(sorted(p.name for p in self.dir.iterdir()), ['missions.yaml'])

    def test_write_failure_is_reported_through_the_callback(self) -> None:
        blocked = self.dir / 'blocked'
        blocked.write_text('a file, not a directory', encoding='utf-8')
        store = MissionYamlStore(str(blocked / 'missions.yaml'), on_write_done=self.on_write_done)

        store.add(rows=['R1'], action='drive')
        self.wait_for_write()

        self.assertTrue(self.statuses[-1].startswith('ERROR:'))


if __name__ == '__main__':
    unittest.main()
