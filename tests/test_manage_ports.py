"""Docker port publication without starting a container."""

import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


class TestDockerPorts(unittest.TestCase):
    def test_default_and_configured_driver_port(self):
        spec = importlib.util.spec_from_file_location(
            'manage', Path(__file__).parents[1] / 'manage.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for configured_port in (None, '9090'):
            with self.subTest(port=configured_port), tempfile.TemporaryDirectory() as directory:
                with patch.object(module.signal, 'signal'):
                    manager = module.DevkitManager()
                manager.root_dir = Path(directory)
                env_file = manager.root_dir / '.env'
                if configured_port:
                    env_file.write_text(f'DEVKIT_DRIVER_UI_PORT={configured_port}\n')
                with patch.object(manager, '_gpu_render_flags', return_value=[]):
                    command = manager._base_docker_cmd(env_file, 'test-dds')
                ports = [command[i + 1] for i, arg in enumerate(command) if arg == '-p']
                self.assertEqual(ports, [
                    '80:80', '8080:8080', '8081:8081', '127.0.0.1:6081:6081',
                    f'{configured_port or "8090"}:{configured_port or "8090"}',
                    '8765:8765', '6080:6080', '8734:8734', '8888:8888',
                ])
                self.assertEqual(command[command.index('--env-file') + 1],
                                 str(env_file) if configured_port else '/dev/null')
