  #!/usr/bin/env python3
"""Regenerate the Gazebo arena from the production occupancy map and arena poses."""

from __future__ import annotations

import math
from pathlib import Path
from xml.sax.saxutils import escape

import cv2
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORK = PACKAGE.parent
MAP = WORK / 'bringup/maps/arena.pgm'
ARENA = WORK / 'mission_manager/config/arena.yaml'
OUTPUT = PACKAGE / 'worlds/arena.sdf'
RESOLUTION = 0.05
ORIGIN_X = -0.611
ORIGIN_Y = -0.555


def box_model(name, xyz, size, color, *, static=True, yaw=0.0):
    x, y, z = xyz
    sx, sy, sz = size
    name = escape(name)
    rgba = ' '.join(str(v) for v in (*color, 1))
    return f'''<model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.7f}</pose>
      <link name="body">
        <collision name="collision"><geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry></collision>
        <visual name="appearance"><geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>
      </link>
    </model>'''


def wall_rectangles(pixels):
    """Coalesce occupied map cells into horizontal rectangles, then vertically."""
    height, width = pixels.shape
    active = {}
    output = []
    for row in range(height + 1):
        runs = []
        if row < height:
            col = 0
            while col < width:
                if pixels[row, col] != 0:
                    col += 1
                    continue
                begin = col
                while col < width and pixels[row, col] == 0:
                    col += 1
                runs.append((begin, col))
        run_set = set(runs)
        for run in set(active) - run_set:
            output.append((*run, *active.pop(run)))
        for run in runs:
            if run not in active:
                active[run] = (row, row + 1)
            else:
                start, _ = active[run]
                active[run] = (start, row + 1)
    return output


def tag_visuals(tag_id):
    """Build the marker from physical black and white squares in the scene."""
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, tag_id, 8, borderBits=1)
    visuals = []
    for row in range(8):
        for col in range(8):
            x = (col - 3.5) * 0.004
            y = (3.5 - row) * 0.004
            value = 0.01 if marker[row, col] == 0 else 0.95
            visuals.append(f'''<visual name="tag_{row}_{col}">
          <pose>{x:.4f} {y:.4f} 0.0202 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>0.004 0.004</size></plane></geometry>
          <material><ambient>{value} {value} {value} 1</ambient>
            <diffuse>{value} {value} {value} 1</diffuse></material>
        </visual>''')
    return ''.join(visuals)


def cube_model(tag_id, x, y, tabletop_z):
    size = 0.04
    mass = 0.04
    inertia = mass * size * size / 6.0
    z = tabletop_z + size / 2
    return f'''<model name="cube_{tag_id}">
      <pose>{x:.5f} {y:.5f} {z:.5f} 0 0 0</pose>
      <link name="body">
        <inertial><mass>{mass}</mass><inertia>
          <ixx>{inertia:.9f}</ixx><iyy>{inertia:.9f}</iyy><izz>{inertia:.9f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia></inertial>
        <collision name="cube"><geometry><box><size>{size} {size} {size}</size></box></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>
        </collision>
        <visual name="cube_appearance"><geometry><box><size>{size} {size} {size}</size></box></geometry>
          <material><ambient>0.85 0.79 0.64 1</ambient><diffuse>0.85 0.79 0.64 1</diffuse></material>
        </visual>
        {tag_visuals(tag_id)}
      </link>
    </model>'''


def container_models(name, x, y, z, color):
    rgba = (0.06, 0.17, 0.85) if color == 'blue' else (0.85, 0.05, 0.04)
    parts = []
    for suffix, dx, dy, sx, sy, sz in (
        ('base', 0, 0, .12, .12, .006),
        ('front', .057, 0, .006, .12, .073),
        ('back', -.057, 0, .006, .12, .073),
        ('left', 0, .057, .12, .006, .073),
        ('right', 0, -.057, .12, .006, .073),
    ):
        cz = z + (sz / 2 if suffix == 'base' else sz / 2)
        parts.append(box_model(f'{name}_{suffix}', (x + dx, y + dy, cz),
                               (sx, sy, sz), rgba))
    return parts


def local_pose(area, forward, lateral):
    x, y, yaw = area['x_m'], area['y_m'], area['yaw_rad']
    return (x + math.cos(yaw) * forward - math.sin(yaw) * lateral,
            y + math.sin(yaw) * forward + math.cos(yaw) * lateral)


def main():
    pixels = cv2.imread(str(MAP), cv2.IMREAD_GRAYSCALE)
    if pixels is None or pixels.shape != (150, 150):
        raise RuntimeError('Mapa arena.pgm ausente ou com dimensões inesperadas.')
    config = yaml.safe_load(ARENA.read_text())
    areas = config['service_areas']
    models = []
    for index, (col0, col1, row0, row1) in enumerate(wall_rectangles(pixels)):
        x = ORIGIN_X + (col0 + col1) * RESOLUTION / 2
        y = ORIGIN_Y + (150 - (row0 + row1) / 2) * RESOLUTION
        models.append(box_model(f'map_wall_{index}', (x, y, .18),
                                ((col1 - col0) * RESOLUTION,
                                 (row1 - row0) * RESOLUTION, .36),
                                (.42, .43, .45)))

    scene = {
        'ws_1': {'cubes': [(1, -.28), (2, -.10), (10, .10)],
                 'containers': [('red', .32)]},
        'ws_2': {'cubes': [(4, -.34), (5, -.18), (3, -.02)],
                 'containers': [('blue', .18), ('red', .37)]},
        'ws_3': {'cubes': [(6, -.28), (12, -.08), (11, .10)],
                 'containers': [('blue', .30)]},
        'ws_4': {'cubes': [(13, -.18)],
                 'containers': [('red', .28)]},
    }
    tag_ids = set()
    for name, area in areas.items():
        tabletop_z = area['height_cm'] / 100.0
        # The near edge is 15 cm ahead of the navigation pose. The 4 cm
        # tabletop is physical geometry and is visible to LiDAR and camera.
        x, y = local_pose(area, .35, 0)
        models.append(box_model(f'{name}_table', (x, y, tabletop_z / 2),
                                (.40, 1.10, tabletop_z),
                                (.91, .91, .89), yaw=area['yaw_rad']))
        for tag_id, lateral in scene[name]['cubes']:
            cx, cy = local_pose(area, .33, lateral)
            models.append(cube_model(tag_id, cx, cy, tabletop_z))
            tag_ids.add(tag_id)
        for color, lateral in scene[name]['containers']:
            cx, cy = local_pose(area, .34, lateral)
            models.extend(container_models(f'{name}_{color}', cx, cy,
                                           tabletop_z, color))

    world = '''<?xml version="1.0"?>
<sdf version="1.9"><world name="arena">
  <physics name="physics" type="ignored"><max_step_size>0.004</max_step_size>
    <real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <gravity>0 0 -9.81</gravity>
  <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose>
    <diffuse>0.85 0.85 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular>
    <direction>-0.4 0.2 -0.9</direction></light>
  <model name="floor"><static>true</static><link name="ground">
    <collision name="ground"><geometry><plane><normal>0 0 1</normal>
      <size>20 20</size></plane></geometry></collision>
    <visual name="floor_appearance"><geometry><plane><normal>0 0 1</normal>
      <size>20 20</size></plane></geometry>
      <material><ambient>0.75 0.75 0.73 1</ambient>
        <diffuse>0.75 0.75 0.73 1</diffuse></material></visual>
  </link></model>
'''
    world += '\n'.join(models)
    world += '\n</world></sdf>\n'
    OUTPUT.write_text(world)
    print(f'{OUTPUT}: {len(models)} modelos, {len(tag_ids)} tags')


if __name__ == '__main__':
    main()
