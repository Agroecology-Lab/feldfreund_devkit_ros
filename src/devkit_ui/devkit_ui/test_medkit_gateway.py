"""Unit tests for the Medkit controls without ROS or web-server startup."""

import ast
import asyncio
import signal
import subprocess
import threading
import unittest
from html import escape
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


class TestMedkitGateway(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        self.process = make_process(pid=137)
        self.popen = Mock(return_value=self.process)
        self.controls = make_controls(self.popen)

    async def test_render_registers_controls_without_starting_a_process(self) -> None:
        self.popen.assert_not_called()
        self.controls.ui.label.assert_called_once_with('')
        self.controls.label.style.assert_called_once_with('color:#57606a')
        self.assertEqual(self.controls.ui.button.call_count, 2)
        self.assertTrue(callable(self.controls.start))
        self.assertTrue(callable(self.controls.stop))

    async def test_start_launches_gateway_on_loopback_and_reports_pid(self) -> None:
        self.controls.start()

        self.popen.assert_called_once_with(
            ['ros2', 'launch', 'ros2_medkit_gateway', 'bringup.launch.py',
             'server_host:=127.0.0.1'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.assert_status('started (pid 137)', 'color:#1a7f37')
        self.process.poll.assert_not_called()
        self.process.wait.assert_not_called()
        self.process.communicate.assert_not_called()
        self.process.terminate.assert_not_called()

    async def test_start_while_running_does_not_launch_or_terminate_again(self) -> None:
        self.controls.start()

        self.controls.start()

        self.popen.assert_called_once()
        self.process.poll.assert_called_once_with()
        self.process.terminate.assert_not_called()
        self.assert_status('already running', 'color:#1a7f37')

    async def test_start_replaces_exited_process_regardless_of_exit_status(self) -> None:
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
                await controls.stop()
                replacement.send_signal.assert_called_once_with(signal.SIGINT)
                old_process.terminate.assert_not_called()

    async def test_launch_errors_are_displayed_and_allow_retry(self) -> None:
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
                await controls.stop()
                process.send_signal.assert_called_once_with(signal.SIGINT)

    async def test_failed_restart_keeps_previous_process_tracked(self) -> None:
        self.controls.start()
        self.process.poll.return_value = 1
        self.popen.side_effect = OSError('cannot launch')

        self.controls.start()

        self.assert_status('ERROR: cannot launch', 'color:#cf222e')
        await self.controls.stop()
        self.process.send_signal.assert_called_once_with(signal.SIGINT)
        self.assert_status('stopped', 'color:#57606a')

    async def test_stop_without_start_is_safe_and_repeatable(self) -> None:
        await self.controls.stop()
        await self.controls.stop()

        self.popen.assert_not_called()
        self.process.terminate.assert_not_called()
        self.assert_status('stopped', 'color:#57606a')

    async def test_stop_sends_sigint_and_waits_before_clearing_handle(self) -> None:
        self.controls.start()

        await self.controls.stop()
        await self.controls.stop()

        self.process.send_signal.assert_called_once_with(signal.SIGINT)
        self.process.wait.assert_called_once_with(timeout=15)
        self.process.terminate.assert_not_called()
        self.process.communicate.assert_not_called()
        self.process.kill.assert_not_called()
        self.assert_status('stopped', 'color:#57606a')

    async def test_start_after_completed_stop_launches_new_process(self) -> None:
        self.controls.start()
        await self.controls.stop()
        replacement = make_process(pid=202)
        self.popen.return_value = replacement

        self.controls.start()

        self.assertEqual(self.popen.call_count, 2)
        self.process.poll.assert_not_called()
        self.assert_status('started (pid 202)', 'color:#1a7f37')
        await self.controls.stop()
        replacement.send_signal.assert_called_once_with(signal.SIGINT)
        self.process.send_signal.assert_called_once_with(signal.SIGINT)

    async def test_stop_after_launch_failure_resets_error_status(self) -> None:
        self.popen.side_effect = FileNotFoundError('ros2 missing')
        self.controls.start()

        await self.controls.stop()

        self.popen.assert_called_once()
        self.process.terminate.assert_not_called()
        self.assert_status('stopped', 'color:#57606a')

    async def test_stop_reaps_already_exited_process(self) -> None:
        self.controls.start()
        self.process.poll.return_value = 0

        await self.controls.stop()

        self.process.send_signal.assert_called_once_with(signal.SIGINT)
        self.assert_status('stopped', 'color:#57606a')

    async def test_signal_failure_keeps_process_for_retry(self) -> None:
        self.controls.start()
        error = PermissionError('cannot signal')
        self.process.send_signal.side_effect = error

        await self.controls.stop()

        self.assert_status(f'ERROR: {error}', 'color:#cf222e')
        self.process.wait.assert_not_called()
        self.controls.start()
        self.popen.assert_called_once()
        self.process.send_signal.side_effect = None
        await self.controls.stop()
        self.assertEqual(self.process.send_signal.call_count, 2)
        self.assert_status('stopped', 'color:#57606a')

    async def test_shutdown_escalates_only_after_each_timeout(self) -> None:
        timeout = subprocess.TimeoutExpired('ros2', 15)
        for waits, expected in (
            ([0], [unittest.mock.call.send_signal(signal.SIGINT), unittest.mock.call.wait(timeout=15)]),
            ([timeout, 0], [unittest.mock.call.send_signal(signal.SIGINT), unittest.mock.call.wait(timeout=15),
                            unittest.mock.call.terminate(), unittest.mock.call.wait(timeout=5)]),
            ([timeout, timeout, 0], [unittest.mock.call.send_signal(signal.SIGINT), unittest.mock.call.wait(timeout=15),
                                     unittest.mock.call.terminate(), unittest.mock.call.wait(timeout=5),
                                     unittest.mock.call.kill(), unittest.mock.call.wait(timeout=5)]),
        ):
            with self.subTest(waits=waits):
                process = make_process(pid=204)
                controls = make_controls(Mock(return_value=process))
                process.wait.side_effect = waits
                controls.start()
                await controls.stop()
                self.assertEqual(process.mock_calls, expected)
                controls.label.set_text.assert_called_with('stopped')
                await controls.stop()
                self.assertEqual(process.mock_calls, expected)

    async def test_final_wait_failure_keeps_handle_for_retry(self) -> None:
        self.controls.start()
        self.process.wait.side_effect = subprocess.TimeoutExpired('ros2', 5)

        await self.controls.stop()

        self.assertTrue(self.controls.label.set_text.call_args.args[0].startswith('ERROR:'))
        self.controls.start()
        self.popen.assert_called_once()
        self.process.wait.side_effect = None
        await self.controls.stop()
        self.assertEqual(self.process.send_signal.call_count, 2)
        self.assert_status('stopped', 'color:#57606a')

    async def test_shutdown_keeps_ui_responsive_and_prevents_overlapping_actions(self) -> None:
        waiting = threading.Event()
        release = threading.Event()

        def wait_for_release(timeout):
            waiting.set()
            if not release.wait(timeout):
                raise subprocess.TimeoutExpired('ros2', timeout)
            return 0

        async def interact_while_stopping():
            try:
                self.assertTrue(await asyncio.to_thread(waiting.wait, 5))
                self.controls.label.set_text.assert_called_with('stopping')
                self.process.poll.return_value = 0
                self.controls.start()
                await self.controls.stop()
                self.popen.assert_called_once()
                self.process.send_signal.assert_called_once_with(signal.SIGINT)
            finally:
                release.set()

        self.controls.start()
        self.process.wait.side_effect = wait_for_release
        await asyncio.gather(self.controls.stop(), interact_while_stopping())
        self.assert_status('stopped', 'color:#57606a')
        self.controls.start()
        self.assertEqual(self.popen.call_count, 2)

    async def test_each_tab_starts_and_stops_only_its_own_process(self) -> None:
        other_process = make_process(pid=203)
        self.popen.side_effect = [self.process, other_process]
        other_controls = make_controls(self.popen)
        self.controls.start()

        await other_controls.stop()

        self.process.terminate.assert_not_called()
        self.assert_status('started (pid 137)', 'color:#1a7f37')
        other_controls.start()
        self.assertEqual(self.popen.call_count, 2)
        other_controls.label.set_text.assert_called_with('started (pid 203)')
        await self.controls.stop()
        self.process.send_signal.assert_called_once_with(signal.SIGINT)
        other_process.terminate.assert_not_called()
        other_controls.start()
        self.assertEqual(self.popen.call_count, 2)
        other_controls.label.set_text.assert_called_with('already running')
        await other_controls.stop()
        other_process.send_signal.assert_called_once_with(signal.SIGINT)

    async def test_gateway_and_sovd_links_use_page_host(self) -> None:
        for hostname, authority in (
            ('robot.local', 'robot.local'), ('192.0.2.10', '192.0.2.10'),
            ('localhost', 'localhost'), ('2001:db8::1', '[2001:db8::1]'),
        ):
            with self.subTest(hostname=hostname):
                controls = make_controls(self.popen, hostname=hostname)
                html = '\n'.join(call.args[0] for call in controls.ui.html.call_args_list)
                self.assertIn(f'href="http://{authority}:8080/"', html)
                self.assertIn(f'href="http://{authority}:3000/"', html)
                self.assertIn(f'<code>http://{authority}:8080</code>', html)
                self.assertIn('docker run -p 3000:80 ghcr.io/selfpatch/sovd_web_ui:latest', html)
                self.assertIn('authentication, TLS, and restricted network access', html)

    async def test_host_is_escaped_in_html(self) -> None:
        controls = make_controls(self.popen, hostname='robot"><img src=x>')
        html = '\n'.join(call.args[0] for call in controls.ui.html.call_args_list)
        self.assertNotIn('<img', html)
        self.assertIn('robot&quot;&gt;&lt;img src=x&gt;', html)

    def assert_status(self, text: str, color: str) -> None:
        self.controls.label.set_text.assert_called_with(text)
        self.controls.label.style.assert_called_with(color)


def make_process(pid: int):
    process = Mock(spec=['pid', 'poll', 'terminate', 'wait', 'communicate', 'kill', 'send_signal'], pid=pid)
    process.poll.return_value = None
    return process


def make_controls(popen, hostname='robot.local'):
    label = Mock(spec=['classes', 'style', 'set_text'])
    label.classes.return_value = label
    label.style.return_value = label
    ui = Mock(spec=['label', 'button', 'html', 'separator', 'context'])
    ui.label.return_value = label
    ui.context.client.request.url.hostname = hostname
    build_controls(ui, SimpleNamespace(
        Popen=popen, DEVNULL=subprocess.DEVNULL, TimeoutExpired=subprocess.TimeoutExpired,
    ))
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
    namespace = {'signal': signal, 'escape': escape,
                 'ng_run': SimpleNamespace(io_bound=asyncio.to_thread)}
    # NOTE: Only the checked-in Medkit block executes, with UI and subprocess doubles.
    exec(  # pylint: disable=exec-used
        compile(ast.fix_missing_locations(factory), source_path, 'exec'), namespace,
    )
    return namespace['build_controls']


build_controls = load_controls_builder()


if __name__ == '__main__':
    unittest.main()
