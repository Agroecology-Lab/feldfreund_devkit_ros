import unittest

from devkit_ui.actions import ACTIONS, action_ros_msgs


class TestActionRosMsgs(unittest.TestCase):
    def test_registry_contains_drive_and_weed(self) -> None:
        self.assertIn('drive', ACTIONS)
        self.assertIn('weed', ACTIONS)

    def test_unknown_action_yields_no_messages(self) -> None:
        self.assertEqual(action_ros_msgs('does_not_exist', None, True), [])

    def test_drive_has_no_tool_topic_so_yields_no_messages(self) -> None:
        self.assertEqual(action_ros_msgs('drive', {}, True), [])
        self.assertEqual(action_ros_msgs('drive', {}, False), [])

    def test_weed_engages_and_disengages_tool_topic(self) -> None:
        topic = ACTIONS['weed'].tool_topic
        assert topic is not None

        self.assertEqual(action_ros_msgs('weed', None, True), [(topic, True)])
        self.assertEqual(action_ros_msgs('weed', None, False), [(topic, False)])


if __name__ == '__main__':
    unittest.main()
