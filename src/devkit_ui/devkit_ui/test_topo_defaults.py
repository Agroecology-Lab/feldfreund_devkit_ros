import unittest
from xml.etree import ElementTree

from devkit_ui.topo_defaults import default_actions, default_definitions


class TestTopoDefaults(unittest.TestCase):
    def test_path_following_stays_outside_the_replanning_rate_limiter(self) -> None:
        """Verify path following is a sibling of the planner rate limiter in both default trees."""
        for name, planner in (('default_bt', 'ComputePathToPose'),
                              ('row_traversal_bt', 'ComputePathThroughPoses')):
            with self.subTest(definition=name):
                tree = ElementTree.fromstring(default_definitions()[name])
                pipeline = tree.find('.//PipelineSequence')
                self.assertEqual([child.tag for child in pipeline],
                                 ['RateController', 'FollowPath'])
                rate, follow = pipeline
                self.assertEqual(float(rate.attrib['hz']), 1.0)
                self.assertEqual(rate.find(f'.//{planner}').attrib['planner_id'], 'GridBased')
                self.assertEqual(follow.attrib['controller_id'], 'FollowPath')

    def test_action_factories_isolate_nested_templates(self) -> None:
        """Verify mutating nested action templates leaves subsequent defaults unchanged."""
        actions = default_actions()
        original = default_actions()

        actions['row_traversal']['action_goal_template']['poses'][0]['header']['frame_id'] = 'other'
        actions['navigate_to_pose']['action_goal_template']['pose']['pose'] = 'other'
        actions.pop('limbic_row_follow')

        self.assertEqual(default_actions(), original)

    def test_definition_factories_isolate_maps(self) -> None:
        """Verify editing or removing copied definitions leaves subsequent defaults unchanged."""
        definitions = default_definitions()
        original = default_definitions()
        definitions['default_bt'] = '<custom/>'
        definitions.pop('row_traversal_bt')
        self.assertEqual(default_definitions(), original)

    def test_actions_use_matching_goal_shapes_and_behavior_trees(self) -> None:
        """Verify each default action matches its server, goal template, and behavior tree."""
        actions = default_actions()
        expected = {
            'navigate_to_pose': ('NavigateToPose', '/navigate_to_pose', False, 'default_bt'),
            'row_traversal': ('NavigateThroughPoses', '/navigate_through_poses', True,
                              'row_traversal_bt'),
            'limbic_row_follow': ('NavigateToPose', '/limbic_row_follow', False, None),
        }
        self.assertEqual(set(actions), set(expected))
        for name, (action_type, server, composable, definition) in expected.items():
            with self.subTest(action=name):
                action = actions[name]
                self.assertEqual(action['action_type'], f'nav2_msgs.action.{action_type}')
                self.assertEqual(action['action_server'], server)
                self.assertIs(action['composable'], composable)
                template = action['action_goal_template']
                pose = {'header': {'frame_id': '${node.nav_frame}'}, 'pose': '${node.pose}'}
                goal = {'poses': [pose]} if composable else {'pose': pose}
                goal['behavior_tree'] = '${definitions.' + definition + '}' if definition else ''
                self.assertEqual(template, goal)
                if definition:
                    self.assertIn(definition, default_definitions())

    def test_behavior_trees_bind_single_and_multiple_goals_correctly(self) -> None:
        """Verify default trees share path bindings and prune passed goals for row traversal."""
        definitions = default_definitions()
        for name, planner, goal_key in (
            ('default_bt', 'ComputePathToPose', 'goal'),
            ('row_traversal_bt', 'ComputePathThroughPoses', 'goals'),
        ):
            with self.subTest(definition=name):
                root = ElementTree.fromstring(definitions[name])
                self.assertEqual(root.attrib['BTCPP_format'], '4')
                tree = root.find(f"BehaviorTree[@ID='{root.attrib['main_tree_to_execute']}']")
                self.assertIsNotNone(tree)
                compute = tree.find(f'.//{planner}')
                self.assertIsNotNone(compute)
                self.assertEqual(compute.attrib[goal_key], '{' + goal_key + '}')
                follow = tree.find('.//FollowPath')
                self.assertEqual(compute.attrib['path'], follow.attrib['path'])
                self.assertEqual(compute.attrib['path'], '{path}')

        sequence = ElementTree.fromstring(definitions['row_traversal_bt']).find(
            './/ReactiveSequence')
        self.assertEqual([child.tag for child in sequence],
                         ['RemovePassedGoals', 'ComputePathThroughPoses'])
        self.assertEqual(sequence[0].attrib['input_goals'], '{goals}')
        self.assertEqual(sequence[0].attrib['output_goals'], '{goals}')
