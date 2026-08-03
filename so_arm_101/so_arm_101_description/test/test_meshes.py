"""Validate binary STL mesh topology using only the standard library."""
from collections import Counter
from pathlib import Path
import struct

import pytest

MESH_DIR = Path(__file__).parent.parent / 'meshes'
VISUAL_DIR = MESH_DIR / 'visual'
COLLISION_DIR = MESH_DIR / 'collision'

MESH_FILES = [
    'base_link.stl', 'link1_1.stl', 'link2_1.stl', 'link3_1.stl',
    'link4_1.stl', 'link5_1.stl', 'clamp_1.stl', 'clamp_2.stl',
]


def load_stl(path):
    """Return triangles from a well-formed binary STL file."""
    data = path.read_bytes()
    assert len(data) >= 84, f'{path.name} is not a binary STL'
    face_count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + face_count * 50, (
        f'{path.name} has an invalid binary STL size')

    triangles = []
    for index in range(face_count):
        values = struct.unpack_from('<12fH', data, 84 + index * 50)
        triangles.append(tuple(
            tuple(values[offset:offset + 3]) for offset in (3, 6, 9)
        ))
    return triangles


def unique_vertices(triangles):
    return {vertex for triangle in triangles for vertex in triangle}


@pytest.mark.parametrize('filename', MESH_FILES)
def test_visual_mesh_loads(filename):
    assert load_stl(VISUAL_DIR / filename), f'{filename} has no faces'


@pytest.mark.parametrize('filename', MESH_FILES)
def test_collision_mesh_loads(filename):
    assert load_stl(COLLISION_DIR / filename), f'{filename} has no faces'


@pytest.mark.parametrize('filename', MESH_FILES)
def test_collision_mesh_is_watertight(filename):
    triangles = load_stl(COLLISION_DIR / filename)
    edge_counts = Counter(
        tuple(sorted(edge))
        for triangle in triangles
        for edge in (
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        )
    )
    assert all(count == 2 for count in edge_counts.values()), (
        f'Collision mesh {filename} is not watertight')


@pytest.mark.parametrize('filename', MESH_FILES)
def test_collision_simpler_than_visual(filename):
    visual_vertices = unique_vertices(load_stl(VISUAL_DIR / filename))
    collision_vertices = unique_vertices(load_stl(COLLISION_DIR / filename))
    assert len(collision_vertices) < len(visual_vertices), (
        f'{filename}: collision ({len(collision_vertices)} verts)'
        f' not simpler than visual ({len(visual_vertices)} verts)'
    )
