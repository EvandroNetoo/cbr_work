"""Reactive PP organization with role-separated perception and two cargo slots."""
import math

from .errors import StepFailed, PrecisionSlotOccupied, PrecisionObjectAlreadyCorrect
from .models import Step, DeliveryOutcome
from .precision_perception import slot_occupant
from .world_state import EMPTY


class PrecisionRuntime:
    def _is_precision_area(self):
        arena = getattr(self, '_arena', None)
        area = arena.service_areas.get(getattr(self, '_current_location', '')) if arena else None
        return area is not None and area.area_type == 'PP'

    def _configure_pp_goal(self, goal):
        if self._is_precision_area():
            config = self._arena.precision_perception
            goal.classify_pp_tags = True
            goal.pp_reference_z_m = config.reference_z_m
            goal.pp_reference_z_tolerance_m = config.reference_z_tolerance_m

    def _pp_scene(self):
        scene = getattr(self, '_last_pp_scene', None)
        if (scene is None or scene[0] != self._current_location
                or self._current_wall_distance_mm is None
                or abs(scene[1] - self._current_wall_distance_mm) > 1
                or abs(scene[2] - self._current_lateral_position_mm) > 1):
            raise StepFailed('PP: ocupação exige observação nova na posição atual.')
        self._pp_update_verified(scene[3], scene[4])
        return scene[3], scene[4]

    def _pp_update_verified(self, references, objects):
        """Recognize correct pairs from every new scene, including search images."""
        if not getattr(self, '_pp_organizing', False):
            return
        final = getattr(self, '_pp_final_state', {})
        verified = getattr(self, '_pp_verified', set())
        _known, held, cargo = self._world_state.snapshot()
        on_robot = {cube for cube in cargo.values() if cube != EMPTY}
        if held != EMPTY:
            on_robot.add(held)
        for target, desired in final.items():
            if target not in references or desired in on_robot:
                continue
            try:
                occupant = slot_occupant(target, references, objects,
                                         self._arena.precision_perception)
            except StepFailed:
                # An ambiguous pair is not evidence of a correct placement.
                verified.discard(target)
                continue
            if occupant == desired:
                verified.add(target)
            elif occupant is not None:
                verified.discard(target)

    def _pp_skip_confirmed_pick(self, tag_id):
        if getattr(self, '_pp_skip_correct_pick_tag', None) != tag_id:
            return
        try:
            self._pp_scene()
        except StepFailed:
            pass
        for target, desired in getattr(self, '_pp_final_state', {}).items():
            if desired == tag_id and target in getattr(self, '_pp_verified', set()):
                raise PrecisionObjectAlreadyCorrect(f'PP: objeto {tag_id} já está no alojamento {target}.')

    def _pp_search_points(self):
        config = self._arena.pickup_recovery
        area = self._arena.service_areas[self._current_location]
        points = []
        for wall, positions in ((area.alignment.distance_mm, config.search_positions_mm),
                                (config.safety_search_distance_mm, config.safety_search_positions_mm)):
            for lateral in sorted(positions, key=lambda p: abs(p - self._current_lateral_position_mm)):
                if (wall, lateral) not in points:
                    points.append((wall, lateral))
        return points

    def _pp_record_reference_view(self, references):
        """Fixed-reference visibility survives cube pickup and cargo transfers."""
        if not hasattr(self, '_pp_reference_views'):
            self._pp_reference_views = {}
        key = (self._current_location, self._current_wall_distance_mm,
               self._current_lateral_position_mm, bool(getattr(self, '_search_led_off', False)))
        self._pp_reference_views[key] = frozenset(references)

    def _pp_reference_view_excludes(self, tag_id, wall, lateral, tolerance=1):
        for (area, observed_wall, observed_lateral, led_off), ids in reversed(
                tuple(getattr(self, '_pp_reference_views', {}).items())):
            if (area == self._current_location
                    and led_off == bool(getattr(self, '_search_led_off', False))
                    and abs(observed_wall - wall) <= tolerance
                    and abs(observed_lateral - lateral) <= tolerance):
                return tag_id not in ids
        return False

    def _pp_move_to_next_search_point(self, tag_id, *, reference, attempted):
        # Each recovery owns its attempts. Object-search history must not suppress
        # a fresh reference search, or searches after a cargo/base movement.
        for wall, lateral in self._pp_search_points():
            if (wall, lateral) in attempted:
                continue
            attempted.add((wall, lateral))
            self._check_canceled()
            self._pp_aligned_reference = None
            self._move_to_table_position(wall, lateral,
                                        f'busca PP da {"referência" if reference else "objeto"} {tag_id}')
            return True
        return False

    def _pp_scan_for(self, tag_id, *, reference, attempted=None, deferred=None):
        """Search all configured viewpoints before reporting a missing tag."""
        attempted = set() if attempted is None else attempted
        while True:
            while self._pp_move_to_next_search_point(tag_id, reference=reference, attempted=attempted):
                self._observe_visit()
                refs, objects = self._pp_scene()
                if tag_id is None or tag_id in (refs if reference else objects):
                    return
            if not deferred:
                break
            # Only after every unexplored viewpoint failed, retry old negatives:
            # pickup can have uncovered a previously occluded fixed reference.
            attempted.difference_update(deferred)
            deferred = None
        raise StepFailed(f'PP: {"referência" if reference else "objeto"} {tag_id} não encontrado após busca.')

    def _pp_destination_is_aligned(self, tag_id):
        aligned = getattr(self, '_pp_aligned_reference', None)
        config = self._arena.pickup_recovery
        return (aligned is not None and aligned[:2] == (self._current_location, tag_id)
                and self._current_wall_distance_mm is not None
                and abs(aligned[2] - self._current_wall_distance_mm) <= config.wall_tolerance_mm
                and abs(aligned[3] - self._current_lateral_position_mm) <= config.travel_tolerance_mm)

    def _pp_seek_reference(self, tag_id):
        attempted = set()
        config = self._arena.pickup_recovery
        memory = getattr(self, '_pp_reference_observations', {}).get((self._current_location, tag_id))
        deferred = set()
        if memory is None:
            for wall, lateral in self._pp_search_points():
                if self._pp_reference_view_excludes(tag_id, wall, lateral):
                    attempted.add((wall, lateral))
            deferred = attempted.copy()
            if self._pp_reference_view_excludes(tag_id, self._current_wall_distance_mm,
                                               self._current_lateral_position_mm):
                self._pp_scan_for(tag_id, reference=True, attempted=attempted, deferred=deferred)
                deferred = set()
                memory = self._pp_reference_observations[(self._current_location, tag_id)]
        while True:
            if memory is not None:
                wall, lateral = self._precision_memory_destination(memory)
                moved = (abs(wall - self._current_wall_distance_mm) > config.wall_tolerance_mm
                         or abs(lateral - self._current_lateral_position_mm) > config.travel_tolerance_mm)
                if moved:
                    self._move_to_table_position(wall, lateral, f'alinhamento PP pela referência {tag_id}')
                    self._observe_visit()
            else:
                wall = lateral = None
            # Reuse an existing scene when no movement/transfer invalidated it.
            try:
                refs, objects = self._pp_scene()
            except StepFailed:
                self._observe_visit()
                refs, objects = self._pp_scene()
            if tag_id not in refs:
                self._pp_aligned_reference = None
                # This camera snapshot already examined the current search point.
                for point in self._pp_search_points():
                    if (abs(point[0] - self._current_wall_distance_mm) <= config.wall_tolerance_mm
                            and abs(point[1] - self._current_lateral_position_mm) <= config.travel_tolerance_mm):
                        attempted.add(point)
                self._pp_scan_for(tag_id, reference=True, attempted=attempted, deferred=deferred)
                deferred = set()
                memory = self._pp_reference_observations[(self._current_location, tag_id)]
                continue
            if memory is None:
                memory = self._pp_reference_observations[(self._current_location, tag_id)]
                continue  # Align once from the newly discovered pose.
            occupant = slot_occupant(tag_id, refs, objects, self._arena.precision_perception)
            if (abs(wall - self._current_wall_distance_mm) <= config.wall_tolerance_mm
                    and abs(lateral - self._current_lateral_position_mm) <= config.travel_tolerance_mm):
                self._pp_aligned_reference = (self._current_location, tag_id,
                                              self._current_wall_distance_mm, self._current_lateral_position_mm)
            else:
                self._pp_aligned_reference = None
            return occupant

    def _pp_find_object(self, tag_id):
        try:
            _refs, objects = self._pp_scene()
        except StepFailed:
            self._observe_visit()
            _refs, objects = self._pp_scene()
        if tag_id in objects:
            return
        if self._position_from_memory(tag_id) is not None:
            self._observe_visit()
            _refs, objects = self._pp_scene()
            if tag_id in objects:
                return
        self._pp_scan_for(tag_id, reference=False)

    def _pp_run_operation(self, goal_handle, plan, visit, action, *, tag_id=None,
                          reference_tag_id=None, slot_id=None):
        self._check_canceled()
        step = Step(f'{visit.visit_id}_pp_{self._completed_steps}_{action}_{tag_id}', action,
                    tag_id=tag_id, reference_tag_id=reference_tag_id, slot_id=slot_id)
        self._report_scheduled_operation(goal_handle, plan, step, step.step_id,
                                         f'PP: {action} objeto {tag_id}, referência {reference_tag_id}')
        self._execute_step(step)
        known, held, cargo = self._world_state.snapshot()
        correct = {'pick': held == tag_id, 'store': held == EMPTY and cargo.get(slot_id) == tag_id,
                   'retrieve': held == tag_id and cargo.get(slot_id) == EMPTY,
                   'place_on_precision_table': held == EMPTY}[action]
        if not known or not correct:
            raise StepFailed(f'PP: efeito físico de {action} não confirmado.')
        self._completed_steps += 1
        self._last_pp_scene = None
        if action == 'place_on_precision_table':
            self._delivery_outcomes.append(DeliveryOutcome(step.step_id, tag_id, visit.target,
                                                          action, action, reference_tag_id=reference_tag_id))

    def _pp_pick_store(self, goal_handle, plan, visit, tag_id, *, observed=False, skip_if_correct=False):
        known, held, cargo = self._world_state.snapshot()
        free = next((slot for slot, tag in cargo.items() if tag == EMPTY), None)
        if not known or held != EMPTY or free is None:
            raise StepFailed('PP: é necessário um compartimento livre antes de retirar o ocupante.')
        if observed:
            try:
                _refs, objects = self._pp_scene()
            except StepFailed:
                objects = {}
            if tag_id not in objects:
                self._pp_find_object(tag_id)
        else:
            self._pp_find_object(tag_id)
        # The immediate pick must consume the supplied pose without normal base alignment.
        self._pp_direct_pick_tag = tag_id
        self._pp_skip_correct_pick_tag = tag_id if skip_if_correct else None
        try:
            self._pp_skip_confirmed_pick(tag_id)
            self._pp_run_operation(goal_handle, plan, visit, 'pick', tag_id=tag_id)
        except PrecisionObjectAlreadyCorrect:
            return None
        finally:
            self._pp_direct_pick_tag = None
            self._pp_skip_correct_pick_tag = None
        self._pp_run_operation(goal_handle, plan, visit, 'store', tag_id=tag_id, slot_id=free)
        self._pp_owned_slots.add(free)
        return free

    def _pp_known_wrong_object(self, destinations, verified):
        """Prioritize a seen wrong cube using coordinates corrected for base motion."""
        config = self._arena.precision_perception
        candidates = sorted(self._tag_observations.items(), key=lambda item: (
            abs(item[1].pickup_lateral_position_mm - self._current_lateral_position_mm), item[0]))
        for (area, tag), obj in candidates:
            if area != self._current_location or tag not in destinations or destinations[tag] in verified:
                continue
            ref = self._pp_reference_observations.get((area, destinations[tag]))
            if ref is None:
                # Its destination is guaranteed to exist; find it after pickup.
                return tag
            # FollowWall lateral is opposite base X; wall distance adds to tag Y.
            op, rp = obj.detection.pose.position, ref.detection.pose.position
            ox = op.x - obj.lateral_position_mm / 1000
            oy = op.y + obj.wall_distance_mm / 1000
            rx = rp.x - ref.lateral_position_mm / 1000 + config.slot_offset_x_m
            ry = rp.y + ref.wall_distance_mm / 1000 + config.slot_offset_y_m
            if math.hypot(ox - rx, oy - ry) > config.occupancy_radius_m:
                return tag
        return None

    def _pp_surplus_destination(self, final):
        """An unrequested cube may use an unspecified cavity, never the tabletop."""
        if len(final) == 7:
            return None  # Every cavity is prescribed; there is no unspecified destination to search.
        refs = self._pp_reference_observations
        candidates = [tag for (area, tag) in refs if area == self._current_location and tag not in final]
        config = self._arena.pickup_recovery
        area = self._arena.service_areas[self._current_location]
        if not candidates:
            for lateral in config.search_positions_mm:
                self._move_to_table_position(area.alignment.distance_mm, lateral, 'busca dos 7 alojamentos PP')
                self._observe_visit()
            candidates = [tag for (area_id, tag) in refs if area_id == self._current_location and tag not in final]
        for reference in candidates:
            if self._pp_seek_reference(reference) is None:
                return reference
        return None

    def _pp_full_cargo_destination(self, carried, destinations):
        """Drain a full cargo into a vacancy before picking another occupant."""
        examined = set()
        for slot, cube in carried:
            target = destinations.get(cube)
            if target is not None:
                examined.add(target)
                if self._pp_seek_reference(target) is None:
                    return slot, cube, target
        # A source cavity was emptied by pickup. It can temporarily buffer a
        # normalized cube when both desired destinations are still occupied.
        attempted = set()
        while True:
            references = list(getattr(self, '_pp_reference_observations', {}))
            for area, target in references:
                if area != self._current_location or target in examined:
                    continue
                examined.add(target)
                if self._pp_seek_reference(target) is None:
                    slot, cube = carried[0]
                    return slot, cube, target
            if not self._pp_move_to_next_search_point(None, reference=True, attempted=attempted):
                raise StepFailed('PP: carga cheia e nenhum alojamento vazio encontrado após busca.')
            self._observe_visit()

    def _run_precision_organization(self, goal_handle, plan, visit):
        known, held, cargo = self._world_state.snapshot()
        if not known or held != EMPTY or sum(tag == EMPTY for tag in cargo.values()) < 2:
            raise StepFailed('Organização PP requer garra vazia e pelo menos dois compartimentos livres.')
        final = dict(visit.pp_final_state)
        destinations = {tag: reference for reference, tag in final.items() if tag is not None}
        verified = set()
        self._pp_final_state = final
        self._pp_verified = verified
        self._pp_owned_slots = set()
        self._pp_aligned_reference = None
        self._pp_organizing = True
        self._pp_reference_views = {}
        try:
            self._observe_visit()
            self._pp_scene()
            # Every successful iteration fixes a prescribed cavity, or drains a surplus cube.
            for _iteration in range(84):
                self._check_canceled()
                _, _, cargo = self._world_state.snapshot()
                carried_items = [(slot, tag) for slot, tag in cargo.items()
                                 if tag != EMPTY and slot in self._pp_owned_slots]
                carried_items.sort(key=lambda item: item[1] not in destinations)
                carried = carried_items[0] if carried_items else None
                if len(verified) == len(final) and not carried_items:
                    return
                if carried:
                    selected_slot, cube = carried
                    target = destinations.get(cube)
                    if (any(tag in destinations for _slot, tag in carried_items)
                            and all(tag != EMPTY for tag in cargo.values())):
                        selected_slot, cube, target = self._pp_full_cargo_destination(
                            carried_items, destinations)
                    if target is None:
                        target = self._pp_surplus_destination(final)
                        if target is None:
                            if len(verified) == len(final):
                                # No unrequested cavity is free: retain surplus internally.
                                return
                            # Keep the surplus on-board and advance the chain into
                            # a fresh confirmed vacancy using the second compartment.
                            cleared_null = False
                            for candidate, desired in final.items():
                                if candidate in verified:
                                    continue
                                occupant = self._pp_seek_reference(candidate)
                                if occupant == desired:
                                    verified.add(candidate)
                                elif desired is None:
                                    self._pp_pick_store(goal_handle, plan, visit, occupant, observed=True)
                                    verified.add(candidate)
                                    cleared_null = True
                                    break
                                elif occupant is None:
                                    target, cube = candidate, desired
                                    selected_slot = self._pp_pick_store(goal_handle, plan, visit, cube)
                                    break
                            if target is None:
                                if len(verified) == len(final):
                                    return
                                # A null cavity just cleared may complete the board on the next pass.
                                if cleared_null:
                                    continue
                                raise StepFailed('PP: objeto excedente armazenado, sem alojamento livre para continuar.')
                else:
                    if len(verified) == len(final):
                        return
                    cube = self._pp_known_wrong_object(destinations, verified)
                    if cube is None:
                        target = next(slot for slot in final if slot not in verified)
                        cube = final[target]
                        if cube is None:
                            occupant = self._pp_seek_reference(target)
                            if occupant is not None:
                                self._pp_pick_store(goal_handle, plan, visit, occupant, observed=True)
                            verified.add(target)
                            continue
                        # No reference search/analysis before picking this cube.
                    else:
                        target = destinations[cube]
                    selected_slot = self._pp_pick_store(goal_handle, plan, visit, cube, skip_if_correct=True)
                    if selected_slot is None:
                        continue  # Search proved this pending cube is already correct.
                occupant = self._pp_seek_reference(target)
                if occupant is not None:
                    if occupant == cube:
                        raise StepFailed(f'PP: objeto {cube} aparece na carga e no destino; detecção ambígua.')
                    self._pp_pick_store(goal_handle, plan, visit, occupant, observed=True)
                    # Its confirmed pickup clears the slot. Retrieve next; the place
                    # action takes the final snapshot and realigns only if cargo access moved the base.
                self._pp_run_operation(goal_handle, plan, visit, 'retrieve', tag_id=cube, slot_id=selected_slot)
                try:
                    self._pp_run_operation(goal_handle, plan, visit, 'place_on_precision_table',
                                           tag_id=cube, reference_tag_id=target)
                except PrecisionSlotOccupied:
                    # Retrieval freed this compartment. Return the held cube to
                    # it, then plan from the new occupancy on the next iteration.
                    verified.discard(target)
                    try:
                        refs, objects = self._pp_scene()
                    except StepFailed:
                        refs = objects = {}
                    for reference in tuple(verified):
                        if reference in refs:
                            try:
                                occupant = slot_occupant(reference, refs, objects,
                                                         self._arena.precision_perception)
                            except StepFailed:
                                verified.discard(reference)
                            else:
                                if occupant != final[reference]:
                                    verified.discard(reference)
                    self._pp_run_operation(goal_handle, plan, visit, 'store',
                                           tag_id=cube, slot_id=selected_slot)
                    continue
                # Trust the physical release result; no post-deposit camera session.
                if target in final and final[target] == cube:
                    verified.add(target)
                else:
                    verified.discard(target)
            raise StepFailed('PP: limite de organização atingido sem confirmar final_state.')
        finally:
            self._pp_organizing = False
            self._pp_final_state = {}
            self._pp_verified = set()
            self._pp_skip_correct_pick_tag = None
            known, held, cargo = self._world_state.snapshot()
            self._pp_inventory_pending = (not known or held != EMPTY
                                          or any(tag != EMPTY for tag in cargo.values()))
