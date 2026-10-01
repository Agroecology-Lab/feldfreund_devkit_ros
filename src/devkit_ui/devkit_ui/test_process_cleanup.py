"""Exercise tool process cleanup without importing ROS or starting the UI."""

import ast
import os
import signal
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock


class TestProcessCleanup(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).with_name('ui_node.py')
        tree = ast.parse(source.read_text(encoding='utf-8'))
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == '_kill_group')
        self.os = mock.Mock(getpgid=mock.Mock(return_value=123))
        namespace = {'os': self.os, 'signal': signal, 'subprocess': subprocess}
        # NOTE: Execute only the checked-in helper with controlled process and OS doubles.
        exec(  # pylint: disable=exec-used
            compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
        self.kill_group = namespace['_kill_group']
        self.proc = mock.Mock(pid=123, poll=mock.Mock(return_value=None))
        self.timeout = subprocess.TimeoutExpired('tool', 0.1)

    def test_no_process_or_already_reaped(self):
        self.kill_group(None)
        self.proc.poll.return_value = 0
        self.kill_group(self.proc)
        self.os.getpgid.assert_not_called()
        self.os.killpg.assert_not_called()

    def test_graceful_exit(self):
        self.kill_group(self.proc, grace=0.1)
        self.os.killpg.assert_called_once_with(123, signal.SIGTERM)
        self.proc.wait.assert_called_once_with(timeout=0.1)

    def test_sigkill_after_timeout(self):
        self.proc.wait.side_effect = [self.timeout, 0]
        self.kill_group(self.proc, grace=0.1)
        self.assertEqual(self.os.killpg.call_args_list,
                         [mock.call(123, signal.SIGTERM), mock.call(123, signal.SIGKILL)])
        self.assertEqual(self.proc.wait.call_args_list,
                         [mock.call(timeout=0.1), mock.call(timeout=2)])

    def test_exit_during_lookup_is_reaped(self):
        self.os.getpgid.side_effect = ProcessLookupError
        self.kill_group(self.proc)
        self.proc.wait.assert_called_once_with(timeout=2)

    def test_exit_during_sigterm_is_reaped(self):
        self.os.killpg.side_effect = ProcessLookupError
        self.kill_group(self.proc)
        self.proc.wait.assert_called_once_with(timeout=2)

    def test_exit_during_sigkill_is_reaped(self):
        self.os.killpg.side_effect = [None, ProcessLookupError]
        self.proc.wait.side_effect = [self.timeout, 0]
        self.kill_group(self.proc, grace=0.1)
        self.assertEqual(self.proc.wait.call_args_list,
                         [mock.call(timeout=0.1), mock.call(timeout=2)])

    def test_final_wait_timeout_is_bounded(self):
        for missing_group in (False, True):
            with self.subTest(missing_group=missing_group):
                self.os.getpgid.side_effect = ProcessLookupError if missing_group else None
                self.proc.wait.reset_mock(side_effect=True)
                self.proc.wait.side_effect = self.timeout
                self.kill_group(self.proc, grace=0.1)
                self.assertEqual(self.proc.wait.call_args, mock.call(timeout=2))

    def test_real_children_are_reaped(self):
        self.kill_group.__globals__['os'] = os
        for ignore_term in (False, True):
            with self.subTest(ignore_term=ignore_term):
                handler = signal.SIG_IGN if ignore_term else signal.SIG_DFL
                code = ('import signal, time; '
                        f'signal.signal(signal.SIGTERM, {int(handler)}); '
                        'print("ready", flush=True); time.sleep(30)')
                with subprocess.Popen([sys.executable, '-c', code], start_new_session=True,
                                      stdout=subprocess.PIPE, text=True) as proc:
                    try:
                        self.assertEqual(proc.stdout.readline().strip(), 'ready')
                        self.kill_group(proc, grace=0.1)
                        self.assertEqual(proc.returncode,
                                         -signal.SIGKILL if ignore_term else -signal.SIGTERM)
                        with self.assertRaises(ChildProcessError):
                            os.waitpid(proc.pid, os.WNOHANG)
                    finally:
                        if proc.poll() is None:
                            proc.kill()
                            proc.wait(timeout=2)
