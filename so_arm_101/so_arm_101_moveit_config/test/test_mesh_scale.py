import re
import struct
from pathlib import Path


PACKAGE = Path(__file__).parents[1]
DESCRIPTION = PACKAGE.parent / 'so_arm_101_description'


def _binary_stl_extent(path):
    data = path.read_bytes()
    triangles = struct.unpack_from('<I', data, 80)[0]
    vertices = []
    for index in range(triangles):
        offset = 84 + index*50 + 12
        vertices.extend(struct.unpack_from('<3f', data, offset + vertex*12)
                        for vertex in range(3))
    return [max(point[axis] for point in vertices)
            - min(point[axis] for point in vertices) for axis in range(3)]


def test_link5_millimetre_mesh_is_scaled_to_plausible_metres():
    xacro = (DESCRIPTION / 'urdf' / 'so_101.urdf.xacro').read_text()
    matches = re.findall(r'link5_1\.stl" scale="([^"]+)"', xacro)
    assert matches == ['0.001 0.001 0.001', '0.001 0.001 0.001']
    raw_extent = _binary_stl_extent(
        DESCRIPTION / 'meshes' / 'collision' / 'link5_1.stl')
    scaled = [value*0.001 for value in raw_extent]
    assert all(0.05 < value < 0.20 for value in scaled)
