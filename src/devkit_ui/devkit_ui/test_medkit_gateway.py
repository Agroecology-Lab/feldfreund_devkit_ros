"""Unit tests for the Medkit controls without ROS or web-server startup."""

import ast
import subprocess
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


class TestMedkitGateway(unittest.TestCase):
    def setUp(self) -> None:
        self.process = make_process(pid=137)
        self.popen = Mock(return_value=self.process)
        self.controls = make_controls(self.popen)

    def test_render_registers_controls_without_starting_a_process(self) -> None:
        self.popen.assert_not_called()
        self.controls.ui.label.assert_called_once_with('')
        self.controls.label.style.assert_called_once_with('color:#57606a')
        self.assertEqual(self.controls.ui.button.call_count, 2)
        self.assertTrue(callable(self.controls.start))
        self.assertTrue(callable(self.controls.stop))

    def test_start_launches_gateway_on_all_interfaces_and_reports_pid(self) -> None:
        self.controls.start()

        self.popen.assert_called_once_with(
            ['ros2', 'launch', 'ros2_medkit_gateway', 'bringup.launch.py',
             'server_host:=0.0.0.0'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.assert_status('started (pid 137)', 'color:#1a7f37')
        self.process.poll.assert_not_called()
        self.process.wait.assert_not_called()
        self.process.communicate.assert_not_called()
        self.process.terminate.assert_not_called()

    def test_start_while_running_does_not_launch_or_terminate_again(self) -> None:
        self.controls.start()

        self.controls.start()

        self.popen.assert_called_once()
        self.process.poll.assert_called_once_with()
        self.process.terminate.assert_not_called()
        self.assert_status('already running', 'color:#1a7f37')

    def test_start_replaces_exited_process_regardless_of_exit_status(self) -> None:
        for returncode in (0, 1, -15):
            with self.subTest(returncode=returncode):
                old_process = make_process(pid=100)
                replacement = make_process(pid=200)
                popen = Mock(side_effect=[old_process, replacement])
                controls = make_controls(popen)
                controls.start()
                old_process.poll.return_value = returncode

                controls.start()

                self.assertEqual(popen.call_count, 2)
                controls.label.set_text.assert_called_with('started (pid 200)')
                controls.label.style.assert_called_with('color:#1a7f37')
                controls.stop()
                replacement.terminate.assert_called_once_with()
                old_process.terminate.assert_not_called()

    def test_launch_errors_are_displayed_and_allow_retry(self) -> None:
        for error in (FileNotFoundError('ros2 missing'), PermissionError('access denied'),
                      OSError('process limit reached')):
            with self.subTest(error=error):
                process = make_process(pid=201)
                popen = Mock(side_effect=[error, process])
                controls = make_controls(popen)

                controls.start()

                controls.label.set_text.assert_called_with(f'ERROR: {error}')
                controls.label.style.assert_called_with('color:#cf222e')
                controls.start()
                self.assertEqual(popen.call_count, 2)
                controls.label.set_text.assert_called_with('started (pid 201)')
                controls.label.style.assert_called_with('color:#1a7f37')
                controls.stop()
                process.terminate.assert_called_once_with()

    def test_failed_restart_keeps_previous_process_tracked(self) -> None:
        self.controls.start()
        self.process.poll.return_value = 1
        self.popen.side_effect = OSError('cannot launch')

        self.controls.start()

        self.assert_status('ERROR: cannot launch', 'color:#cf222e')
        self.controls.stop()
        self.process.terminate.assert_called_once_with()
        self.assert_status('stopped', 'color:#57606a')

    def test_stop_without_start_is_safe_and_repeatable(self) -> None:
        self.controls.stop()
        self.controls.stop()

        self.popen.assert_not_called()
        self.process.terminate.assert_not_called()
        self.assert_status('stopped', 'color:#57606a')

    def test_stop_terminates_once_without_waiting_and_clears_handle(self) -> None:
        self.controls.start()

        self.controls.stop()
        self.controls.stop()

        self.process.terminate.assert_called_once_with()
        self.process.wait.assert_not_called()
        self.process.communicate.assert_not_called()
        self.process.kill.assert_not_called()
        self.assert_status('stopped', 'color:#57606a')

    def test_start_after_stop_launches_new_process_even_if_old_one_is_still_exiting(self) -> None:
        self.controls.start()
        self.controls.stop()
        replacement = make_process(pid=202)
        self.popen.return_value = replacement

        self.controls.start()

        self.assertEqual(self.popen.call_count, 2)
        self.process.poll.assert_not_called()
        self.assert_status('started (pid 202)', 'color:#1a7f37')
        self.controls.stop()
        replacement.terminate.assert_called_once_with()
        self.process.terminate.assert_called_once_with()

    def test_stop_after_launch_failure_resets_error_status(self) -> None:
        self.popen.side_effect = FileNotFoundError('ros2 missing')
        self.controls.start()

        self.controls.stop()

        self.popen.assert_called_once()
        self.process.terminate.assert_not_called()
        self.assert_status('stopped', 'color:#57606a')

    def test_stop_requests_termination_even_when_process_has_exited(self) -> None:
        self.controls.start()
        self.process.poll.return_value = 0

        self.controls.stop()

        self.process.terminate.assert_called_once_with()
        self.assert_status('stopped', 'color:#57606a')

    def test_termination_failure_propagates_without_losing_process_or_status(self) -> None:
        self.controls.start()
        error = PermissionError('cannot terminate')
        self.process.terminate.side_effect = error
        self.controls.label.reset_mock()

        with self.assertRaises(PermissionError) as raised:
            self.controls.stop()

        self.assertIs(raised.exception, error)
        self.controls.label.set_text.assert_not_called()
        self.controls.label.style.assert_not_called()
        self.controls.start()
        self.popen.assert_called_once()
        self.controls.label.set_text.assert_called_with('already running')
        self.process.terminate.side_effect = None
        self.controls.stop()
        self.assertEqual(self.process.terminate.call_count, 2)
        self.assert_status('stopped', 'color:#57606a')

    def test_each_tab_starts_and_stops_only_its_own_process(self) -> None:
        other_process = make_process(pid=203)
        self.popen.side_effect = [self.process, other_process]
        other_controls = make_controls(self.popen)
        self.controls.start()

        other_controls.stop()

        self.process.terminate.assert_not_called()
        self.assert_status('started (pid 137)', 'color:#1a7f37')
        other_controls.start()
        self.assertEqual(self.popen.call_count, 2)
        other_controls.label.set_text.assert_called_with('started (pid 203)')
        self.controls.stop()
        self.process.terminate.assert_called_once_with()
        other_process.terminate.assert_not_called()
        other_controls.start()
        self.assertEqual(self.popen.call_count, 2)
        other_controls.label.set_text.assert_called_with('already running')
        other_controls.stop()
        other_process.terminate.assert_called_once_with()

    def test_gateway_and_host_web_ui_connection_instructions_are_rendered(self) -> None:
        html = '\n'.join(call.args[0] for call in self.controls.ui.html.call_args_list)

        self.assertIn('href="http://localhost:8080/"', html)
        self.assertIn('href="http://localhost:3000/"', html)
        self.assertIn('docker run -p 3000:80 ghcr.io/selfpatch/sovd_web_ui:latest', html)
        self.assertIn('<code>http://localhost:8080</code>', html)

    def assert_status(self, text: str, color: str) -> None:
        self.controls.label.set_text.assert_called_with(text)
        self.controls.label.style.assert_called_with(color)


def make_process(pid: int):
    process = Mock(spec=['pid', 'poll', 'terminate', 'wait', 'communicate', 'kill'], pid=pid)
    process.poll.return_value = None
    return process


def make_controls(popen):
    label = Mock(spec=['classes', 'style', 'set_text'])
    label.classes.return_value = label
    label.style.return_value = label
    ui = Mock(spec=['label', 'button', 'html', 'separator'])
    ui.label.return_value = label
    build_controls(ui, SimpleNamespace(Popen=popen, DEVNULL=subprocess.DEVNULL))
    buttons = {call.args[0]: call.kwargs['on_click'] for call in ui.button.call_args_list}
    return SimpleNamespace(
        ui=ui, label=label,
        start=buttons['Start Medkit Gateway'], stop=buttons['Stop Medkit Gateway'],
    )


def load_controls_builder():
    """Execute the real Medkit block in a function to preserve per-tab closures.

    Follow the AST harness used by test_navigation_goals, keeping the original
    status initialization and button registration as well as both callbacks.
    """
    source_path = Path(__file__).with_name('ui_node.py')
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'NiceGuiNode'
    )
    method = next(
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name == '_system_content'
    )
    tools_row = next(
        node for node in ast.walk(method)
        if isinstance(node, ast.With) and any(
            isinstance(statement, ast.AnnAssign)
            and isinstance(statement.target, ast.Name)
            and statement.target.id == '_medkit_proc'
            for statement in node.body
        )
    )
    boundaries = {
        statement.target.id: index for index, statement in enumerate(tools_row.body)
        if isinstance(statement, ast.AnnAssign) and isinstance(statement.target, ast.Name)
    }
    factory = ast.parse('def build_controls(ui, subprocess): pass')
    factory.body[0].body = tools_row.body[
        boundaries['_medkit_proc']:boundaries['_gazebo_proc']
    ]
    namespace = {}
    # NOTE: Only the checked-in Medkit block executes, with UI and subprocess doubles.
    exec(  # pylint: disable=exec-used
        compile(ast.fix_missing_locations(factory), source_path, 'exec'), namespace,
    )
    return namespace['build_controls']


build_controls = load_controls_builder()


if __name__ == '__main__':
    unittest.main()
