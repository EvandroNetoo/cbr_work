"""Geração isolada por CLI, com preferências e artefatos locais por execução."""
from __future__ import annotations

import json
import os
import re
import tempfile
from datetime import datetime
from pathlib import Path
import signal
import subprocess
import sys
from uuid import uuid4

from domain import MISSIONS, TZ, import_scenario

WORKSPACE = Path(__file__).resolve().parents[4]
PLANS_DIRECTORY = WORKSPACE / 'src/cbr_work/mission_manager/config/plans'
INSTALLED_PLANS_DIRECTORY = WORKSPACE / 'install/mission_manager/share/mission_manager/config/plans'
MISSION_PLAN_IDS = {
    'BMT': '1_basic_manipulation_test',
    'BTT1': '2_basic_transportation_test_i',
    'BTT2': '3_basic_transportation_test_ii',
    'AMT': '4_advanced_transportation_test',
    'ATT1': '5_advanced_transportation_test_i',
    'ATT2': '6_advanced_transportation_test_ii',
    'FINAL': '7_final_test',
}
DEFAULTS = {
    'mission': 'ATT2', 'seed': '42', 'decoys': '',
    'commander': str(Path.home() / 'projects/atwork_commander'),
    'python': sys.executable, 'metaarena': '',
    'setup': str(WORKSPACE / 'install/setup.bash'),
    'arena': str(WORKSPACE / 'src/cbr_work/mission_manager/config/arena.yaml'),
    'map': str(WORKSPACE / 'src/cbr_work/bringup/maps/arena.yaml'),
    'planner_params': str(WORKSPACE / 'src/cbr_work/mission_manager/config/mission_planner.yaml'),
    'objective': 'visits_then_distance', 'cargo_slots': 'left right',
    'time_limit': '60', 'max_states': '500000',
    'plans_directory': str(PLANS_DIRECTORY),
}


def save_plan(directory, destination, *, installed_directory=None):
    """Replace the mission's fixed plan, keeping filename and plan_id aligned."""
    directory = Path(directory)
    status = read_status(directory)
    if status['state'] != 'done':
        raise ValueError('Aguarde a conclusão do plano antes de salvar.')
    request = json.loads((directory / 'request.json').read_text(encoding='utf-8'))
    try:
        plan_id = MISSION_PLAN_IDS[request['mission']]
    except KeyError as exc:
        raise ValueError('Tipo de missão inválido.') from exc
    content = (directory / status['plan']).read_text(encoding='utf-8')
    content, count = re.subn(r'^plan_id:[^\n]*', 'plan_id: ' + plan_id, content, flags=re.MULTILINE)
    if count != 1:
        raise ValueError('O plano deve conter exatamente um plan_id no nível principal.')
    targets = [Path(destination).expanduser().resolve()]
    if installed_directory is not None:
        installed = Path(installed_directory).expanduser().resolve()
        if installed not in targets:
            targets.append(installed)
    paths = []
    for folder in targets:
        folder.mkdir(parents=True, exist_ok=True)
        target = folder / (plan_id + '.yaml')
        with tempfile.NamedTemporaryFile(dir=folder, suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content.encode('utf-8'))
        try:
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
        paths.append(target)
    return plan_id, paths


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
    temporary.replace(path)


def read_status(directory):
    directory = Path(directory)
    status = json.loads((directory / 'status.json').read_text(encoding='utf-8'))
    worker = directory / 'worker.json'
    if status['state'] == 'running' and worker.exists():
        pid = json.loads(worker.read_text())['pid']
        try:
            os.kill(pid, 0)
            stat = Path(f'/proc/{pid}/stat')
            dead = stat.exists() and stat.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
        except ProcessLookupError:
            dead = True
        if dead:
            # Re-read after checking the process to avoid overwriting its final status.
            status = json.loads((directory / 'status.json').read_text(encoding='utf-8'))
            if status['state'] == 'running':
                status = {'state': 'error', 'message': 'Processo interrompido. Consulte os logs e gere novamente.'}
                write_json(directory / 'status.json', status)
    return status


def validate_settings(settings):
    if settings['mission'] not in MISSIONS:
        raise ValueError('Prova inválida.')
    try:
        int(settings['seed'])
        if settings['decoys'].strip() and int(settings['decoys']) < 0:
            raise ValueError()
        limit = float(settings['time_limit'])
        if not 0 < limit <= 3600:
            raise ValueError()
        if int(settings['max_states']) < 1:
            raise ValueError()
    except ValueError as exc:
        raise ValueError('Informe seed inteira, decoys não negativos, limite de busca entre 0 e 3600 segundos e máximo de estados positivo.') from exc
    if not settings['cargo_slots'].split():
        raise ValueError('Informe os compartimentos de carga.')
    for key in ('python', 'setup', 'arena', 'map', 'planner_params'):
        if not Path(settings[key]).expanduser().is_file():
            raise ValueError(f'Arquivo não encontrado ({key}): {settings[key]}')
    if not (Path(settings['commander']).expanduser() / 'main.py').is_file():
        raise ValueError('A pasta do Commander precisa conter main.py.')
    if settings['metaarena'].strip() and not Path(settings['metaarena']).expanduser().is_file():
        raise ValueError('Arquivo de metaarena não encontrado.')


def start_generation(root, settings):
    validate_settings(settings)
    directory = Path(root).resolve() / (datetime.now(TZ).strftime('%Y%m%d-%H%M%S') + '_' + uuid4().hex[:8])
    directory.mkdir(parents=True)
    write_json(directory / 'request.json', dict(settings))
    write_json(directory / 'status.json', {'state': 'running', 'message': 'Iniciando geração…'})
    try:
        with (directory / 'worker.log').open('w') as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), str(directory)],
                                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        write_json(directory / 'worker.json', {'pid': process.pid})
    except OSError as exc:
        write_json(directory / 'status.json', {'state': 'error', 'message': str(exc)})
        raise
    return directory


def run_command(command, cwd, log_path, timeout):
    with log_path.open('w') as log:
        process = subprocess.Popen(command, cwd=cwd, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                   env={**os.environ, 'ROS_LOG_DIR': str(log_path.parent / 'ros_logs')})
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise ValueError(f'Tempo máximo excedido. Consulte {log_path.name}.') from exc
    if code:
        raise ValueError(f'CLI terminou com código {code}. Consulte {log_path.name}.')


def generate(directory):
    directory = Path(directory)
    settings = json.loads((directory / 'request.json').read_text())
    try:
        validate_settings(settings)
        commander = Path(settings['commander']).expanduser().resolve()
        command = [str(Path(settings['python']).expanduser()), str(commander / 'main.py')]
        if settings['metaarena'].strip():
            command += ['--metaarena', str(Path(settings['metaarena']).expanduser().resolve())]
        command += ['gerar', settings['mission'], '--seed', str(int(settings['seed'])), '--saida', str(directory)]
        if settings['decoys'].strip():
            command += ['--decoys', str(int(settings['decoys']))]
        write_json(directory / 'status.json', {'state': 'running', 'message': 'Gerando cenário no Commander…'})
        run_command(command, commander, directory / 'commander.log', 120)
        team = directory / f"{settings['mission']}_seed{int(settings['seed'])}_equipe.yaml"
        import_scenario(team.read_text(encoding='utf-8'), team.name)
        plan = directory / 'plano.yaml'
        command = ['ros2', 'run', 'mission_manager', 'generate_plan', '--task', str(team),
                   '--output', str(plan), '--objective', settings['objective'],
                   '--time-limit', settings['time_limit'], '--max-states', settings['max_states'],
                   '--cache', str(directory / 'nav2_distances.json')]
        for key, flag in [('arena', '--arena'), ('map', '--map'), ('planner_params', '--planner-params')]:
            command += [flag, str(Path(settings[key]).expanduser().resolve())]
        command += ['--cargo-slots', *settings['cargo_slots'].split()]
        # All user values are positional arguments, never shell source code.
        command = ['bash', '-c', 'source "$1" && shift && exec "$@"', 'dashboard',
                   str(Path(settings['setup']).expanduser().resolve()), *command]
        write_json(directory / 'status.json', {'state': 'running', 'message': 'Calculando distâncias e otimizando o plano…'})
        run_command(command, WORKSPACE, directory / 'planner.log', 1800 + float(settings['time_limit']))
        if not plan.is_file():
            raise ValueError('O planejador não produziu o plano esperado.')
        write_json(directory / 'status.json', {'state': 'done', 'message': 'Cenário e plano gerados.',
                                              'team': team.name, 'plan': plan.name, 'seed': settings['seed']})
    except Exception as exc:
        write_json(directory / 'status.json', {'state': 'error', 'message': str(exc)})


if __name__ == '__main__':
    generate(Path(sys.argv[1]).resolve())
