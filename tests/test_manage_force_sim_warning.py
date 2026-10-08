"""FORCE_SIM must never silently discard hardware that was detected."""

import contextlib
import importlib.util
import io
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import pytest


class _Stop(Exception):
    """Raised to halt run() once the sim decision has been logged."""


def _run_output(env_text, extra_args=()):
    spec = importlib.util.spec_from_file_location(
        'manage', Path(__file__).parents[1] / 'manage.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as directory, patch.object(module.signal, 'signal'):
        manager = module.DevkitManager()
        manager.root_dir = Path(directory)
        (manager.root_dir / '.env').write_text(env_text)
        out = io.StringIO()
        with contextlib.redirect_stdout(out), \
                patch.object(module.subprocess, 'run', side_effect=_Stop), \
                patch.object(module.os, 'system'):
            with contextlib.suppress(_Stop):
                manager.run(list(extra_args))
        return out.getvalue()


class TestForceSimWarning(unittest.TestCase):
    def setUp(self):
        """Keep a caller's FORCE_SIM setting from altering the test scenarios."""
        environment = patch.dict(os.environ, {'FORCE_SIM': ''})
        environment.start()
        self.addCleanup(environment.stop)

    def test_warns_when_force_sim_overrides_detected_hardware(self):
        out = _run_output('FORCE_SIM=1\nGPS_PORT_ROVER=/dev/ttyACM0\nMCU_PORT=/dev/ttyACM1\n')
        self.assertIn('detected hardware', out)
        self.assertIn('/dev/ttyACM0', out)
        self.assertIn('Sim: TRUE', out)

    def test_no_warning_without_hardware(self):
        out = _run_output('FORCE_SIM=1\nGPS_PORT_ROVER=virtual\nMCU_PORT=virtual\n')
        self.assertNotIn('IGNORED', out)

    def test_no_warning_when_hardware_runs_for_real(self):
        out = _run_output('GPS_PORT_ROVER=/dev/ttyACM0\nMCU_PORT=/dev/ttyACM1\n')
        self.assertNotIn('IGNORED', out)
        self.assertIn('Sim: FALSE', out)


@pytest.mark.parametrize('source', ['file', 'environment', 'cli'])
@pytest.mark.parametrize(('rover', 'mcu'), [
    ('/dev/ttyACM0', 'virtual'),
    ('virtual', '/dev/ttyACM1'),
    ('/dev/ttyACM0', '/dev/ttyACM1'),
    ('virtual', 'virtual'),
])
def test_warning_for_each_force_source_and_hardware_combination(monkeypatch, source, rover, mcu):
    """Each supported override warns exactly once whenever either hardware port is ignored."""
    monkeypatch.delenv('FORCE_SIM', raising=False)
    config = f'GPS_PORT_ROVER={rover}\nMCU_PORT={mcu}\n'
    if source == 'file':
        config += 'FORCE_SIM=1\n'
    elif source == 'environment':
        config += 'FORCE_SIM=0\n'
        monkeypatch.setenv('FORCE_SIM', '1')
    args = ('--sim',) if source == 'cli' else ()

    output = _run_output(config, args)

    assert 'Sim: TRUE' in output
    warnings = [line for line in output.splitlines() if 'IGNORED' in line]
    if rover == mcu == 'virtual':
        assert warnings == []
    else:
        assert len(warnings) == 1
        assert 'WARN' in warnings[0]
        assert f'rover={rover}' in warnings[0]
        assert f'mcu={mcu}' in warnings[0]
