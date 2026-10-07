"""Expand the real robot model to verify the optional Gazebo camera.

Requires xacro (provided by the ROS image, or install with ``pip install xacro``).
"""

from pathlib import Path
from xml.etree import ElementTree

import pytest

xacro = pytest.importorskip('xacro', reason='Camera model tests require the ROS xacro package')
MODEL = Path(__file__).resolve().parents[2] / 'src/devkit_simulation/urdf/sowbot_01.xacro'


@pytest.mark.parametrize('mappings,width,height,rate', [
    ({}, '320', '240', '10'),
    ({'use_camera': 'true'}, '320', '240', '10'),
    ({'use_camera': '1'}, '320', '240', '10'),
    (
        {'camera_width': '640', 'camera_height': '480', 'camera_rate': '29.97'},
        '640', '480', '29.97',
    ),
    ({'camera_width': '1', 'camera_height': '2', 'camera_rate': '0.5'}, '1', '2', '0.5'),
    ({'camera_width': '800'}, '800', '240', '10'),
    ({'camera_height': '600'}, '320', '600', '10'),
    ({'camera_rate': '5'}, '320', '240', '5'),
])
def test_camera_dimensions_and_rate(mappings, width, height, rate):
    robot = expand(mappings)
    sensors = robot.findall(".//sensor[@type='camera']")
    assert len(sensors) == 1
    sensor = sensors[0]
    assert sensor.get('name') == 'camera_sensor'
    assert sensor.findtext('camera/image/width') == width
    assert sensor.findtext('camera/image/height') == height
    assert sensor.findtext('update_rate') == rate
    assert sensor.findtext('topic') == 'camera'
    assert sensor.findtext('gz_frame_id') == 'camera_link'
    assert sensor.findtext('camera/image/format') == 'R8G8B8'


@pytest.mark.parametrize('disabled', ['false', '0'])
def test_camera_disabled_preserves_robot_and_other_sensors(disabled):
    enabled = expand({})
    robot = expand({
        'use_camera': disabled, 'camera_width': '800', 'camera_height': '600', 'camera_rate': '30',
    })
    assert robot.findall(".//sensor[@type='camera']") == []
    assert robot.find("link[@name='camera_link']") is not None
    for tag in ('link', 'joint'):
        assert [element.attrib for element in robot.findall(tag)] == [
            element.attrib for element in enabled.findall(tag)
        ]
    other_sensors = [sensor.attrib for sensor in enabled.findall('.//sensor')
                     if sensor.get('type') != 'camera']
    assert other_sensors
    assert [sensor.attrib for sensor in robot.findall('.//sensor')] == other_sensors


def test_disabling_camera_does_not_leak_into_next_expansion():
    assert expand({'use_camera': 'false'}).find(".//sensor[@type='camera']") is None
    camera = expand({}).find(".//sensor[@type='camera']")
    assert camera is not None
    assert camera.findtext('camera/image/width') == '320'


def expand(mappings):
    return ElementTree.fromstring(xacro.process_file(str(MODEL), mappings=mappings).toxml())
