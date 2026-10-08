"""FORCE_SIM must never silently discard hardware that was detected."""

import contextlib
import importlib.util
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


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
