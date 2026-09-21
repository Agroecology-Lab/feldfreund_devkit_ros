import unittest
from typing import cast

from devkit_ui.missions.sqlite import MissionSqliteStore


class TestMissionSqliteStore(unittest.TestCase):
    def setUp(self) -> None:
        self._store = MissionSqliteStore(':memory:')

    def tearDown(self) -> None:
        self._store.close()

    def test_missions_getter_returns_all_missions_in_id_order(self) -> None:
        self._store.add(rows=[3], action='spray', action_params={'dose': 3}, name='THIRD')
        self._store.add(rows=[1], action='weed', action_params={'dose': 1}, name='FIRST')
        self._store.add(rows=[2], action='seed', action_params={'dose': 2}, name='SECOND')

        missions = self._store.missions

        self.assertEqual(len(missions), 3)
        self.assertEqual([mission['id'] for mission in missions], ['MISSION_1', 'MISSION_2', 'MISSION_3'])
        self.assertEqual([mission['name'] for mission in missions], ['THIRD', 'FIRST', 'SECOND'])
        self.assertEqual([mission['action'] for mission in missions], ['spray', 'weed', 'seed'])
        self.assertEqual([mission['rows'] for mission in missions], [[3], [1], [2]])
        self.assertEqual(
            [mission['action_params'] for mission in missions],
            [{'dose': 3}, {'dose': 1}, {'dose': 2}],
        )

    def test_add_and_find_round_trip(self) -> None:
        mission_id = self._store.add(
            rows=[1, 3, 5],
            action='spray',
            action_params={'dose': 2},
            repeat_every_hours=12,
            active=True,
        )

        self.assertEqual(mission_id, 'MISSION_1')
        mission = self._store.find(mission_id)
        self.assertIsNotNone(mission)
        mission = cast(dict, mission)
        self.assertEqual(mission['id'], mission_id)
        self.assertEqual(mission['name'], mission_id)
        self.assertEqual(mission['rows'], [1, 3, 5])
        self.assertEqual(mission['action'], 'spray')
        self.assertEqual(mission['action_params'], {'dose': 2})
        self.assertEqual(mission['repeat_every_hours'], 12)
        self.assertTrue(mission['active'])
        self.assertTrue(mission['created_at'])
        self.assertIsNone(mission['last_run_at'])
        self.assertIsNone(mission['last_run_success'])


    def test_find_by_name_returns_matching_mission(self) -> None:
        self._store.add(
            rows=[],
            action='noop',
            action_params={},
            name='CUSTOM_NAME',
            active=True,
        )

        mission = self._store.find_by_name('CUSTOM_NAME')
        self.assertIsNotNone(mission)
        mission = cast(dict, mission)
        self.assertEqual(mission['name'], 'CUSTOM_NAME')
        self.assertEqual(mission['id'], 'MISSION_1')


    def test_run_history_is_persisted_and_can_be_cleared(self) -> None:
        mission_id = cast(str, self._store.add(rows=[1], action='drive'))

        self._store.update(mission_id, last_run_at='2026-01-01T10:00:00Z', last_run_success=True)
        mission = cast(dict, self._store.find(mission_id))
        self.assertEqual(mission['last_run_at'], '2026-01-01T10:00:00Z')
        self.assertIs(mission['last_run_success'], True)

        self._store.update(mission_id, last_run_success=False)
        mission = cast(dict, self._store.find(mission_id))
        self.assertIs(mission['last_run_success'], False)

        self._store.update(mission_id, last_run_at=None, last_run_success=None)
        mission = cast(dict, self._store.find(mission_id))
        self.assertIsNone(mission['last_run_at'])
        self.assertIsNone(mission['last_run_success'])

    def test_update_allows_partial_mutable_fields(self) -> None:
        mission_id = self._store.add(
            rows=[1],
            action='old_action',
            action_params={'x': 1},
            name='OLD',
            repeat_every_hours=4,
            active=True,
        )
        self.assertIsNotNone(mission_id)
        mission_id = cast(str, mission_id)

        updated = self._store.update(
            mission_id,
            name='NEW',
            rows=[2, 4],
            action='new_action',
            action_params={'x': 2, 'y': 3},
            repeat_every_hours=8,
            active=False,
            id='MISSION_999',
        )
        self.assertTrue(updated)

        mission = self._store.find(mission_id)
        self.assertIsNotNone(mission)
        mission = cast(dict, mission)
        self.assertEqual(mission['id'], mission_id)
        self.assertEqual(mission['name'], 'NEW')
        self.assertEqual(mission['rows'], [2, 4])
        self.assertEqual(mission['action'], 'new_action')
        self.assertEqual(mission['action_params'], {'x': 2, 'y': 3})
        self.assertEqual(mission['repeat_every_hours'], 8)
        self.assertFalse(mission['active'])


    def test_delete_removes_mission(self) -> None:
        mission_id = self._store.add(rows=[], action='noop')
        self.assertIsNotNone(mission_id)
        mission_id = cast(str, mission_id)

        deleted = self._store.delete(mission_id)
        self.assertTrue(deleted)
        self.assertIsNone(self._store.find(mission_id))


if __name__ == '__main__':
    unittest.main()
