"""PP roles and fresh cavity occupancy, independent of AprilTag IDs."""
import math

from .errors import ConfigurationError, StepFailed


def validate_final_state(state):
    if not state or len(state) > 7:
        raise ConfigurationError('final_state deve definir de 1 a 7 alojamentos da PP.')
    for slot, cube in state.items():
        if isinstance(slot, bool) or not isinstance(slot, int) or slot < 0:
            raise ConfigurationError('final_state: referência deve ser ID inteiro não negativo.')
        if cube is not None and (isinstance(cube, bool) or not isinstance(cube, int) or cube < 0):
            raise ConfigurationError('final_state: objeto deve ser ID inteiro não negativo ou null.')
    cubes = [cube for cube in state.values() if cube is not None]
    if len(cubes) != len(set(cubes)):
        raise ConfigurationError('final_state: IDs de objetos duplicados.')


def is_reference(detection, config):
    p = detection.pose.position
    if not all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
        raise StepFailed('PP: detecção com posição não finita.')
    if detection.header.frame_id != 'arm_base_link':
        raise StepFailed('PP: classificação exige detecção em arm_base_link.')
    return abs(float(p.z) - config.reference_z_m) <= config.reference_z_tolerance_m + 1e-9


def split_detections(detections, config):
    references, objects = {}, {}
    for tag in detections:
        role = references if is_reference(tag, config) else objects
        if int(tag.id) in role:
            raise StepFailed(f'PP: mais de uma tag {tag.id} com a mesma função na cena.')
        role[int(tag.id)] = tag
    return references, objects


def slot_occupant(reference_id, references, objects, config, *, held_tag_id=None):
    """Missing reference is unknown, never evidence of an empty cavity."""
    if reference_id not in references:
        raise StepFailed(f'PP: referência {reference_id} ausente na observação atual.')
    ref = references[reference_id].pose.position
    x, y = ref.x + config.slot_offset_x_m, ref.y + config.slot_offset_y_m
    near = [tag_id for tag_id, tag in objects.items() if tag_id != held_tag_id
            and math.hypot(tag.pose.position.x - x, tag.pose.position.y - y)
            <= config.occupancy_radius_m]
    if len(near) > 1:
        raise StepFailed(f'PP: ocupação ambígua no alojamento {reference_id}: {near}.')
    return near[0] if near else None
