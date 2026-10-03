"""Single source of truth for the BT definitions and actions of every new map.

Maps are created in the field and are not expected to carry settings over
from earlier maps. Anything that creates a map (the UI seeding path, the
clear-and-reseed path and get_maize_topo.py) takes its definitions and actions
from here, never from a previous map or from the LCAS test fixture.
"""
import copy

BT_TO_POSE = """<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
"""

# NavigateThroughPoses sets {goals}, not {goal}.
BT_THROUGH_POSES = """<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ReactiveSequence>
          <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
          <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
        </ReactiveSequence>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
"""

DEFINITIONS = {
    'default_bt': BT_TO_POSE,
    'row_traversal_bt': BT_THROUGH_POSES,
}

ACTIONS = {
    # Headland moves.
    'navigate_to_pose': {
        'composable': False,
        'action_type': 'nav2_msgs.action.NavigateToPose',
        'action_server': '/navigate_to_pose',
        'action_goal_template': {
            'pose': {'header': {'frame_id': '${node.nav_frame}'},
                     'pose': '${node.pose}'},
            'behavior_tree': '${definitions.default_bt}',
        },
    },
    # Geometry-only row driving. Works with no crop present (seeding).
    # Composable: consecutive edges merge into one multi-waypoint goal.
    'row_traversal': {
        'composable': True,
        'action_type': 'nav2_msgs.action.NavigateThroughPoses',
        'action_server': '/navigate_through_poses',
        'action_goal_template': {
            'poses': [{'header': {'frame_id': '${node.nav_frame}'},
                       'pose': '${node.pose}'}],
            'behavior_tree': '${definitions.row_traversal_bt}',
        },
    },
    # Vision-driven row following. The server ignores the BT.
    'limbic_row_follow': {
        'composable': False,
        'action_type': 'nav2_msgs.action.NavigateToPose',
        'action_server': '/limbic_row_follow',
        'action_goal_template': {
            'pose': {'header': {'frame_id': '${node.nav_frame}'},
                     'pose': '${node.pose}'},
            'behavior_tree': '',
        },
    },
}


def default_definitions() -> dict:
    return copy.deepcopy(DEFINITIONS)


def default_actions() -> dict:
    return copy.deepcopy(ACTIONS)
