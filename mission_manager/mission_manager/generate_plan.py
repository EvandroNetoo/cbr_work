"""CLI: Commander team task → validated, optimized mission v2 YAML."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import re
import sys
import tempfile

import yaml

from .errors import ConfigurationError
from .loaders import load_arena, load_plan, validate_plan
from .nav2_distances import Nav2Distances
from .plan_optimizer import import_commander, optimize
from .planner_process import MissionPlannerProcess


def default_file(package, relative):
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory(package)) / relative)
    except (ImportError, LookupError):
        return str(Path(__file__).resolve().parents[2] / package / relative)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--task', type=Path, required=True, help='YAML *_equipe.yaml do Commander')
    result.add_argument('--arena', type=Path, default=default_file('mission_manager', 'config/arena.yaml'))
    result.add_argument('--map', type=Path, default=default_file('bringup', 'maps/arena.yaml'),
                        help='Mapa YAML local (padrão: bringup/maps/arena.yaml)')
    result.add_argument('--keepout-mask', type=Path,
                        help='Máscara local; padrão: <nome_do_mapa>_keepout.yaml na mesma pasta')
    result.add_argument('--planner-params', type=Path,
                        default=default_file('mission_manager', 'config/mission_planner.yaml'))
    result.add_argument('--external-planner', action='store_true',
                        help='Usa somente um planner já iniciado, sem subir o launch automaticamente')
    result.add_argument('--output', type=Path, required=True, help='YAML de saída; não sobrescreve sem --force')
    result.add_argument('--plan-id', help='Padrão: nome do arquivo de saída sem extensão')
    result.add_argument('--objective', choices=['visits', 'distance', 'visits_then_distance'],
                        default='visits_then_distance',
                        help='visits e visits_then_distance desempatam por distância; distance desempata por visitas')
    result.add_argument('--cache', type=Path, default=Path(os.environ.get('XDG_CACHE_HOME',
                        str(Path.home() / '.cache'))) / 'mission_manager/nav2_distances.json')
    result.add_argument('--cargo-slots', nargs='+', default=['left', 'right'])
    result.add_argument('--pp-reference', action='append', default=[], metavar='TAG=REFERENCE',
                        help='Sobrescreve a referência PP de um cubo (padrão: mesmo ID)')
    result.add_argument('--planner-id', default='', help='Plugin Nav2; vazio usa o único plugin disponível')
    result.add_argument('--path-action', default='/mission_planning/compute_path_to_pose')
    result.add_argument('--map-server', default='/mission_planning/map_server')
    result.add_argument('--planner-server', default='/mission_planning/planner_server')
    result.add_argument('--costmap-node', default='/mission_planning/global_costmap/global_costmap')
    result.add_argument('--costmap-service', default='/mission_planning/global_costmap/get_costmap')
    result.add_argument('--keepout-server', default='/mission_planning/keepout_filter_mask_server')
    result.add_argument('--filter-info-server', default='/mission_planning/keepout_costmap_filter_info_server')
    result.add_argument('--timeout', type=float, default=30.0, help='Timeout por chamada ROS, em segundos')
    result.add_argument('--time-limit', type=float, default=60.0, help='Limite da busca, após calcular distâncias')
    result.add_argument('--max-states', type=int, default=500000)
    result.add_argument('--force', action='store_true')
    return result


def references(values):
    output = {}
    for value in values:
        if not re.fullmatch(r'[0-9]+=[0-9]+', value):
            raise ConfigurationError('--pp-reference deve ter formato TAG=REFERENCE.')
        tag, reference = map(int, value.split('='))
        if tag in output:
            raise ConfigurationError(f'--pp-reference repetido para tag {tag}.')
        output[tag] = reference
    return output


class _PlanDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, indentless=False)

    def represent_mapping(self, tag, mapping, flow_style=None):
        if isinstance(mapping, dict) and 'action' in mapping:
            flow_style = True
        return super().represent_mapping(tag, mapping, flow_style=flow_style)


def serialize_plan(document, *, source_task=None):
    """Keep visits in blocks and action dictionaries on a single line."""
    header = ''
    if source_task is not None:
        lines = ['Objetos por área — estados declarados no arquivo de equipe do Commander.']
        for area in source_task['active_service_areas']:
            initial = source_task['start_state'][area]
            final = source_task['finish_state'][area]
            line = f"{area}: inicial={initial['obj_ids']}; final={final['obj_ids']}"
            if initial.get('cont_colors') or final.get('cont_colors'):
                line += (f"; contêineres iniciais={initial.get('cont_colors', [])}"
                         f"; contêineres finais={final.get('cont_colors', [])}")
            lines.append(line)
        header = '\n'.join('# ' + line for entry in lines for line in entry.splitlines()) + '\n\n'
    return header + yaml.dump(document, Dumper=_PlanDumper, sort_keys=False,
                              allow_unicode=True, default_flow_style=False, width=10000)


def write_validated(document, output, arena, cargo_slots, *, force=False, source_task=None):
    """Validate the exact serialized artifact before publishing it atomically."""
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', dir=output.parent,
                                     prefix='.mission_plan_', delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(serialize_plan(document, source_task=source_task))
            stream.flush()
            plan = load_plan(temporary, cargo_capacity=len(cargo_slots))
            validate_plan(plan, arena, tuple(cargo_slots))
            if force:
                temporary.replace(output)
            else:
                # Atomic no-clobber publication, including concurrent CLI runs.
                os.link(temporary, output)
            return plan
        finally:
            temporary.unlink(missing_ok=True)


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    cli = parser()
    command_args = sys.argv if argv is None else ['generate_plan', *argv]
    args = cli.parse_args(remove_ros_args(args=command_args)[1:])
    node = None
    initialized = False
    planner_process = None
    try:
        if args.output.exists() and not args.force:
            raise ConfigurationError(f'Saída já existe: {args.output}. Use --force para sobrescrever.')
        if args.output.suffix != '.yaml':
            raise ConfigurationError('--output deve ter extensão .yaml para seleção pelo executor.')
        if args.output.resolve() in {args.task.resolve(), args.arena.resolve(), args.map.resolve(),
                                     args.cache.resolve()}:
            raise ConfigurationError('--output não pode sobrescrever uma entrada ou o cache.')
        plan_id = args.plan_id or args.output.stem
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', plan_id):
            raise ConfigurationError('plan_id deve começar por letra/número e conter apenas ASCII, _ ou -.')
        if args.output.stem != plan_id:
            raise ConfigurationError('--plan-id deve corresponder ao nome do YAML de saída, sem extensão.')
        if (not math.isfinite(args.timeout) or not math.isfinite(args.time_limit)
                or args.timeout <= 0 or args.time_limit <= 0 or args.max_states < 1
                or len(set(args.cargo_slots)) != len(args.cargo_slots)
                or any(not slot for slot in args.cargo_slots)):
            raise ConfigurationError('Timeouts, limites e cargo-slots inválidos.')
        arena = load_arena(args.arena)
        source_task = yaml.safe_load(args.task.read_text())
        mission = import_commander(source_task, arena, references(args.pp_reference))
        if mission.organization and len(args.cargo_slots) < 2:
            raise ConfigurationError('AMT com ocupação descoberta exige pelo menos dois compartimentos.')
        # Keep Python's Ctrl+C exception active during the CPU-only optimizer too.
        rclpy.init(args=command_args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('mission_plan_generator')
        planner_process = MissionPlannerProcess(node, args)
        planner_process.start()
        distances = Nav2Distances(
            node, arena, args.map, args.cache, planner_id=args.planner_id,
            action_name=args.path_action, map_server=args.map_server,
            planner_server=args.planner_server, costmap_node=args.costmap_node,
            costmap_service=args.costmap_service, keepout_server=args.keepout_server,
            filter_info_server=args.filter_info_server, timeout=args.timeout)
        # Calculate each directed pair once, outside the search time budget.
        pairs = [('start', target) for target in mission.targets]
        pairs += [(a, b) for a in mission.targets for b in mission.targets if a != b]
        pairs += [(target, 'finish') for target in mission.targets]
        matrix = {(a, b): distances.distance(a, b) for a, b in pairs}
        distances.verify_costmap()
        result = optimize(mission, lambda a, b: matrix[a, b], plan_id=plan_id,
                          objective=args.objective, cargo_capacity=len(args.cargo_slots),
                          time_limit=args.time_limit, max_states=args.max_states)
        if not node.context.ok():
            raise KeyboardInterrupt
        plan = write_validated(result.document, args.output, arena, args.cargo_slots,
                               force=args.force, source_task=source_task)
        print(f'Plano: {args.output}\nVisitas: {len(plan.visits)} + finish\n'
              f'Distância Nav2 estimada: {result.distance_m:.3f} m\n'
              f'Objetivo: {args.objective}; ótimo comprovado para este modelo\n'
              f'Estados expandidos: {result.expanded_states}\n'
              f'Cache: {distances.cache.hits} reutilizadas, {distances.cache.misses} calculadas')
        return 0
    except (ConfigurationError, OSError, ValueError, RuntimeError, yaml.YAMLError) as exc:
        if initialized and not rclpy.ok():
            print('Geração cancelada.', file=sys.stderr)
            return 130
        print(f'Erro: {exc}', file=sys.stderr)
        return 2
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Geração cancelada.', file=sys.stderr)
        return 130
    finally:
        if planner_process is not None:
            planner_process.close()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
