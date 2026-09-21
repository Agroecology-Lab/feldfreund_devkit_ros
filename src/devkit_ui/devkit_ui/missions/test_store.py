import sqlite3
import tempfile
import unittest
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import cast
from unittest.mock import Mock, patch

import yaml

from devkit_ui.missions.store import DUE_FAILED, DUE_NOW, MissionStore, validate_mission
from devkit_ui.time_utils import parse_ts

NOW = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)


class TestValidateMission(unittest.TestCase):
    def test_valid_mission_has_no_error(self) -> None:
        self.assertIsNone(validate_mission('NORTH_FIELD', ['R1'], 'drive', 6))

    def test_valid_mission_without_name_or_repeat_has_no_error(self) -> None:
        self.assertIsNone(validate_mission('', ['R1'], 'weed', None))

    def test_no_rows_is_rejected(self) -> None:
        self.assertEqual(validate_mission('A', [], 'drive', None), 'ERROR: no rows selected')

    def test_unknown_action_is_rejected(self) -> None:
        self.assertIn('unknown action', cast(str, validate_mission('A', ['R1'], 'teleport', None)))

    def test_non_integer_repeat_is_rejected(self) -> None:
        self.assertIn('must be an integer', cast(str, validate_mission('A', ['R1'], 'drive', 'often')))  # type: ignore[arg-type]

    def test_non_positive_repeat_is_rejected(self) -> None:
        self.assertIn('must be > 0', cast(str, validate_mission('A', ['R1'], 'drive', 0)))
        self.assertIn('must be > 0', cast(str, validate_mission('A', ['R1'], 'drive', -3)))

    def test_uncleaned_name_is_rejected(self) -> None:
        self.assertIn('invalid name', cast(str, validate_mission('not clean', ['R1'], 'drive', None)))


class TestMissionStoreEditing(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = self.enterContext(tempfile.TemporaryDirectory())  # pylint: disable=consider-using-with
        self.node = SimpleNamespace()
        self.store = MissionStore(str(Path(self.tmp) / 'missions.db'))
        self.addCleanup(self.store.close)
        self.store.attach(self.node)

    def test_unsupported_file_extension_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            MissionStore('/tmp/missions.txt')

    def test_close_releases_the_sqlite_connection(self) -> None:
        self.store.close()

        with self.assertRaises(sqlite3.ProgrammingError):
            self.store.find('MISSION_1')

    def test_close_is_a_no_op_for_the_yaml_backend(self) -> None:
        MissionStore(str(Path(self.tmp) / 'missions.yaml')).close()

    def test_attach_initialises_node_state(self) -> None:
        self.assertEqual(self.node.missions, ())
        self.assertEqual(self.node.missions_version, 1)
        self.assertEqual(self.node.mission_status, '')

    def test_add_cleans_the_name_and_mirrors_state_onto_the_node(self) -> None:
        mission_id = self.store.add(rows=['R1', 'R2'], action='drive', name=' north field! ')

        self.assertEqual(mission_id, 'MISSION_1')
        mission = self.store.find_by_name('NORTH_FIELD')
        assert mission is not None
        self.assertEqual(mission['id'], 'MISSION_1')
        self.assertEqual(mission['rows'], ['R1', 'R2'])
        self.assertEqual([m['id'] for m in self.node.missions], ['MISSION_1'])
        self.assertEqual(self.node.missions_version, 2)

    def test_add_rejects_invalid_mission_and_reports_why(self) -> None:
        self.assertIsNone(self.store.add(rows=[], action='drive'))

        self.assertEqual(self.node.mission_status, 'ERROR: no rows selected')
        self.assertEqual(self.node.missions, ())
        self.assertEqual(self.node.missions_version, 1)

    def test_delete_removes_the_mission(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))

        self.assertTrue(self.store.delete(mission_id))

        self.assertIsNone(self.store.find(mission_id))
        self.assertEqual(self.node.missions, ())

    def test_delete_unknown_mission_reports_not_found(self) -> None:
        self.assertFalse(self.store.delete('MISSION_42'))

        self.assertIn('not found', self.node.mission_status)

    def test_update_changes_mutable_fields(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive', name='OLD'))

        self.assertTrue(self.store.update(mission_id, name='new name', rows=['R2', 'R3'], repeat_every_hours=8))

        mission = self.store.find(mission_id)
        assert mission is not None
        self.assertEqual(mission['name'], 'NEW_NAME')
        self.assertEqual(mission['rows'], ['R2', 'R3'])
        self.assertEqual(mission['repeat_every_hours'], 8)

    def test_update_with_blank_name_falls_back_to_the_id(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive', name='CUSTOM'))

        self.assertTrue(self.store.update(mission_id, name='!!!'))

        mission = self.store.find(mission_id)
        assert mission is not None
        self.assertEqual(mission['name'], mission_id)

    def test_update_rejects_immutable_fields(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))

        self.assertFalse(self.store.update(mission_id, created_at='2020-01-01T00:00:00Z'))

        self.assertIn('cannot update', self.node.mission_status)

    def test_update_unknown_mission_reports_not_found(self) -> None:
        self.assertFalse(self.store.update('MISSION_42', name='X'))

        self.assertIn('not found', self.node.mission_status)

    def test_update_that_fails_validation_is_rejected(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))

        self.assertFalse(self.store.update(mission_id, rows=[]))

        self.assertEqual(self.node.mission_status, 'ERROR: no rows selected')
        mission = self.store.find(mission_id)
        assert mission is not None
        self.assertEqual(mission['rows'], ['R1'])

    def test_set_active_toggles_the_mission(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))

        self.assertTrue(self.store.set_active(mission_id, False))
        mission = self.store.find(mission_id)
        assert mission is not None
        self.assertFalse(mission['active'])

        self.assertTrue(self.store.set_active(mission_id, True))
        mission = self.store.find(mission_id)
        assert mission is not None
        self.assertTrue(mission['active'])

    def test_unknown_mission_has_no_due_time(self) -> None:
        self.assertIsNone(self.store.next_due_in_hours('MISSION_42'))


class TestMissionStoreRunHistory(unittest.TestCase):
    def setUp(self) -> None:
        tmp = self.enterContext(tempfile.TemporaryDirectory())  # pylint: disable=consider-using-with
        self.node = SimpleNamespace()
        self.store = MissionStore(str(Path(tmp) / 'missions.db'))
        self.addCleanup(self.store.close)
        self.store.attach(self.node)

    def test_successful_one_shot_run_is_recorded_and_deactivates_the_mission(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))

        self.assertTrue(self.store.record_run(mission_id, True))

        mission = cast(dict, self.store.find(mission_id))
        self.assertIs(mission['last_run_success'], True)
        self.assertIsNotNone(parse_ts(mission['last_run_at']))
        self.assertFalse(mission['active'])
        self.assertEqual(self.store.today_queue(), [])
        self.assertIsNone(self.store.next_due_in_hours(mission_id))

    def test_failed_run_keeps_the_mission_active_and_queued_for_retry(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))

        self.assertTrue(self.store.record_run(mission_id, False))

        mission = cast(dict, self.store.find(mission_id))
        self.assertIs(mission['last_run_success'], False)
        self.assertTrue(mission['active'])
        self.assertEqual(self.store.today_queue(), [(mission_id, 'R1', 'drive', {})])
        self.assertEqual(self.store.next_due_in_hours(mission_id), DUE_FAILED)

    def test_successful_recurring_run_stays_active_and_waits_for_its_interval(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive', repeat_every_hours=6))

        self.assertTrue(self.store.record_run(mission_id, True))

        mission = cast(dict, self.store.find(mission_id))
        self.assertTrue(mission['active'])
        self.assertEqual(self.store.today_queue(), [])
        self.assertAlmostEqual(cast(float, self.store.next_due_in_hours(mission_id)), 6.0, places=1)

    def test_record_run_mirrors_state_onto_the_node(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))
        version = self.node.missions_version

        self.store.record_run(mission_id, True)

        self.assertEqual(self.node.missions_version, version + 1)
        self.assertIs(self.node.missions[0]['last_run_success'], True)

    def test_record_run_for_unknown_mission_reports_not_found(self) -> None:
        self.assertFalse(self.store.record_run('MISSION_42', True))

        self.assertIn('not found', self.node.mission_status)

    def test_reset_rearms_a_completed_mission(self) -> None:
        mission_id = cast(str, self.store.add(rows=['R1'], action='drive'))
        self.store.record_run(mission_id, True)
        version = self.node.missions_version

        self.assertTrue(self.store.reset(mission_id))

        mission = cast(dict, self.store.find(mission_id))
        self.assertTrue(mission['active'])
        self.assertIsNone(mission['last_run_at'])
        self.assertIsNone(mission['last_run_success'])
        self.assertEqual(self.store.next_due_in_hours(mission_id), DUE_NOW)
        self.assertEqual(self.node.missions_version, version + 1)

    def test_reset_for_unknown_mission_reports_not_found(self) -> None:
        self.assertFalse(self.store.reset('MISSION_42'))

        self.assertIn('not found', self.node.mission_status)


class TestMissionStoreScheduling(unittest.TestCase):
    """Scheduling reads last_run_* fields, so the missions are loaded from a prepared YAML file."""

    def setUp(self) -> None:
        tmp = self.enterContext(tempfile.TemporaryDirectory())  # pylint: disable=consider-using-with
        path = Path(tmp) / 'missions.yaml'
        path.write_text(yaml.safe_dump({'missions': [
            {'id': 'NEVER_RAN', 'rows': ['R1', 'R2'], 'action': 'weed', 'action_params': {'rpm': 3},
             'repeat_every_hours': 6},
            {'id': 'FAILED', 'rows': ['R3'], 'action': 'drive',
             'last_run_at': '2026-01-01T11:00:00Z', 'last_run_success': False, 'repeat_every_hours': 6},
            {'id': 'RECURRING_WAITING', 'rows': ['R4'], 'action': 'drive', 'repeat_every_hours': 6,
             'last_run_at': '2026-01-01T10:00:00Z', 'last_run_success': True},
            {'id': 'RECURRING_ELAPSED', 'rows': ['R5'], 'action': 'drive', 'repeat_every_hours': 6,
             'last_run_at': '2026-01-01T05:00:00Z', 'last_run_success': True},
            {'id': 'MALFORMED_TIMESTAMP', 'rows': ['R9'], 'action': 'drive',
             'last_run_at': {'timestamp': '2026-01-01T05:00:00Z'}, 'last_run_success': True},
            {'id': 'ONE_SHOT_DONE', 'rows': ['R6'], 'action': 'drive',
             'last_run_at': '2026-01-01T09:00:00Z', 'last_run_success': True},
            {'id': 'INACTIVE', 'rows': ['R7'], 'action': 'drive', 'active': False},
            {'id': 'BAD_ACTION', 'rows': ['R8'], 'action': 'teleport'},
        ]}), encoding='utf-8')
        self.store = MissionStore(str(path))
        self.addCleanup(self.store.close)
        self.store.attach(Mock())  # the YAML store logs load results through node.get_logger()
        patcher = patch('devkit_ui.missions.store.now_utc', return_value=NOW)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_today_queue_lists_only_due_active_missions_with_valid_actions(self) -> None:
        queue = self.store.today_queue()

        self.assertEqual(queue, [
            ('NEVER_RAN', 'R1', 'weed', {'rpm': 3}),
            ('NEVER_RAN', 'R2', 'weed', {'rpm': 3}),
            ('FAILED', 'R3', 'drive', {}),
            ('RECURRING_ELAPSED', 'R5', 'drive', {}),
            ('MALFORMED_TIMESTAMP', 'R9', 'drive', {}),
        ])

    def test_never_run_mission_is_due_now(self) -> None:
        self.assertEqual(self.store.next_due_in_hours('NEVER_RAN'), DUE_NOW)

    def test_failed_mission_reports_the_retry_sentinel(self) -> None:
        self.assertEqual(self.store.next_due_in_hours('FAILED'), DUE_FAILED)

    def test_recurring_mission_reports_hours_until_the_interval_elapses(self) -> None:
        self.assertAlmostEqual(cast(float, self.store.next_due_in_hours('RECURRING_WAITING')), 4.0)

    def test_recurring_mission_past_its_interval_is_due_now(self) -> None:
        self.assertEqual(self.store.next_due_in_hours('RECURRING_ELAPSED'), DUE_NOW)

    def test_malformed_timestamp_is_treated_as_never_run(self) -> None:
        self.assertIn(('MALFORMED_TIMESTAMP', 'R9', 'drive', {}), self.store.today_queue())
        self.assertEqual(self.store.next_due_in_hours('MALFORMED_TIMESTAMP'), DUE_NOW)

    def test_completed_one_shot_has_no_due_time(self) -> None:
        self.assertIsNone(self.store.next_due_in_hours('ONE_SHOT_DONE'))

    def test_inactive_mission_has_no_due_time(self) -> None:
        self.assertIsNone(self.store.next_due_in_hours('INACTIVE'))


if __name__ == '__main__':
    unittest.main()
