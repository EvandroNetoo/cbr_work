"""Validate mesh files with trimesh."""
import os

import pytest
import trimesh

MESH_DIR = os.path.join(os.path.dirname(__file__), '..', 'meshes')
VISUAL_DIR = os.path.join(MESH_DIR, 'visual')
COLLISION_DIR = os.path.join(MESH_DIR, 'collision')

MESH_FILES = [
    'base_link.stl', 'link1_1.stl', 'link2_1.stl', 'link3_1.stl',
    'link4_1.stl', 'link5_1.stl', 'clamp_1.stl', 'clamp_2.stl',
]


@pytest.mark.parametrize('filename', MESH_FILES)
def test_visual_mesh_loads(filename):
    mesh = trimesh.load(os.path.join(VISUAL_DIR, filename))
    assert len(mesh.faces) > 0, f"{filename} has no faces"


@pytest.mark.parametrize('filename', MESH_FILES)
def test_collision_mesh_loads(filename):
    mesh = trimesh.load(os.path.join(COLLISION_DIR, filename))
    assert len(mesh.faces) > 0, f"{filename} has no faces"


@pytest.mark.parametrize('filename', MESH_FILES)
def test_collision_mesh_is_watertight(filename):
    mesh = trimesh.load(os.path.join(COLLISION_DIR, filename))
    assert mesh.is_watertight, f"Collision mesh {filename} is not watertight"


@pytest.mark.parametrize('filename', MESH_FILES)
def test_collision_simpler_than_visual(filename):
    vis = trimesh.load(os.path.join(VISUAL_DIR, filename))
    col = trimesh.load(os.path.join(COLLISION_DIR, filename))
    assert len(col.vertices) < len(vis.vertices), (
        f"{filename}: collision ({len(col.vertices)} verts)"
        f" not simpler than visual ({len(vis.vertices)} verts)"
    )
