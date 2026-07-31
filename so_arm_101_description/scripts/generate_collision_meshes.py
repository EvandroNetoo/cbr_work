#!/usr/bin/env python3
"""Generate simplified convex hull collision meshes from visual STL files.

Usage:
    python scripts/generate_collision_meshes.py

Requires: trimesh (pip install trimesh)
"""
import os
import sys


def generate_collision_meshes(
    visual_dir: str = 'meshes/visual',
    collision_dir: str = 'meshes/collision',
) -> int:
    try:
        import trimesh
    except ImportError:
        print('ERROR: trimesh is required. Install with: pip install trimesh')
        return 1

    os.makedirs(collision_dir, exist_ok=True)

    stl_files = [f for f in os.listdir(visual_dir) if f.endswith('.stl')]
    if not stl_files:
        print(f'No STL files found in {visual_dir}')
        return 1

    for filename in sorted(stl_files):
        visual_path = os.path.join(visual_dir, filename)
        collision_path = os.path.join(collision_dir, filename)

        mesh = trimesh.load(visual_path)
        hull = mesh.convex_hull
        hull.export(collision_path)

        v_size = os.path.getsize(visual_path)
        c_size = os.path.getsize(collision_path)
        ratio = c_size / v_size * 100
        status = 'OK' if ratio < 80 else 'WARN'
        print(f'{status}: {filename} visual={v_size:,}B collision={c_size:,}B ({ratio:.0f}%)')

    print(f'\nGenerated {len(stl_files)} collision meshes in {collision_dir}/')
    return 0


if __name__ == '__main__':
    sys.exit(generate_collision_meshes())
