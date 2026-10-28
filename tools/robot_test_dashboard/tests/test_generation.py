from pathlib import Path
import sys
import time

import pytest
from streamlit.testing.v1 import AppTest

from generation import DEFAULTS, read_status, start_generation
from storage import Store

APP = Path(__file__).resolve().parents[1] / 'app.py'
FIXTURE = Path(__file__).parent / 'fixtures/FINAL_seed8_equipe.yaml'


def fake_settings(tmp_path):
    commander = tmp_path / 'commander with spaces'
    commander.mkdir()
    (commander / 'main.py').write_text('''import pathlib, sys
args = sys.argv
out = pathlib.Path(args[args.index('--saida') + 1])
(out / 'FINAL_seed8_equipe.yaml').write_text(%r)
(out / 'FINAL_seed8_juiz.yaml').write_text('montagem: teste')
''' % FIXTURE.read_text())
    binary = tmp_path / 'ros2'
    binary.write_text('#!/bin/bash\nwhile [ "$#" -gt 0 ]; do\n  if [ "$1" = "--output" ]; then shift; printf \'plan_id: plano\\n\' > "$1"; exit 0; fi\n  shift\ndone\nexit 2\n')
    binary.chmod(0o755)
    setup = tmp_path / 'setup with spaces.bash'
    setup.write_text('export PATH="' + str(tmp_path) + ':$PATH"\n')
    return {**DEFAULTS, 'mission': 'FINAL', 'seed': '8', 'commander': str(commander),
            'python': sys.executable, 'setup': str(setup)}


def wait_job(directory):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        status = read_status(directory)
        if status['state'] != 'running':
            return status
        time.sleep(.05)
    pytest.fail('Worker did not finish')


def test_background_pipeline_preserves_artifacts_and_unique_runs(tmp_path):
    settings = fake_settings(tmp_path)
    first = start_generation(tmp_path / 'jobs', settings)
    status = wait_job(first)
    assert status['state'] == 'done', status
    assert (first / status['plan']).read_text() == 'plan_id: plano\n'
    assert (first / 'FINAL_seed8_juiz.yaml').is_file()
    assert (first / 'commander.log').is_file()
    second = start_generation(tmp_path / 'jobs', settings)
    assert second != first
    assert wait_job(second)['state'] == 'done'


def test_failed_planner_keeps_scenario_and_log(tmp_path):
    settings = fake_settings(tmp_path)
    (tmp_path / 'ros2').write_text('#!/bin/bash\necho falha_planejamento\nexit 7\n')
    directory = start_generation(tmp_path / 'jobs', settings)
    status = wait_job(directory)
    assert status['state'] == 'error'
    assert '7' in status['message']
    assert (directory / 'FINAL_seed8_equipe.yaml').is_file()
    assert 'falha_planejamento' in (directory / 'planner.log').read_text()


def test_invalid_seed_does_not_start_worker(tmp_path):
    settings = fake_settings(tmp_path)
    settings['seed'] = '8; touch bad'
    with pytest.raises(ValueError, match='seed inteira'):
        start_generation(tmp_path / 'jobs', settings)
    assert not (tmp_path / 'jobs').exists()


def test_generation_fields_survive_new_browser_session(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_TEST_DB', str(tmp_path / 'tests.sqlite3'))
    app = AppTest.from_file(str(APP)).run()
    next(e for e in app.radio if e.label == 'Origem do cenário').set_value('Gerar por seed').run()
    app.text_input(key='generation:seed').input('123').run()
    app.text_input(key='generation:commander').input('/tmp/meu commander').run()
    assert not app.exception
    assert Store().generation_settings()['seed'] == '123'
    reopened = AppTest.from_file(str(APP)).run()
    next(e for e in reopened.radio if e.label == 'Origem do cenário').set_value('Gerar por seed').run()
    assert reopened.text_input(key='generation:seed').value == '123'
    assert reopened.text_input(key='generation:commander').value == '/tmp/meu commander'
    assert not reopened.exception


def test_completed_generation_can_create_attached_draft(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_TEST_DB', str(tmp_path / 'tests.sqlite3'))
    settings = fake_settings(tmp_path)
    directory = start_generation(tmp_path / 'generations', settings)
    assert wait_job(directory)['state'] == 'done'
    app = AppTest.from_file(str(APP)).run()
    next(e for e in app.radio if e.label == 'Origem do cenário').set_value('Gerar por seed').run()
    assert not app.exception
    next(e for e in app.button if e.label == 'Criar rascunho com cenário e plano').click().run()
    assert not app.exception
    assert not app.error
    row = Store().list_latest()[0]
    assert row['payload']['seed'] == '8'
    assert row['payload']['plan']['text'] == 'plan_id: plano\n'
    assert row['payload']['scenario']['file']['text'] == FIXTURE.read_text()


def test_interrupted_worker_does_not_block_future_generations(tmp_path):
    from generation import write_json
    write_json(tmp_path / 'status.json', {'state': 'running', 'message': 'aguardando'})
    write_json(tmp_path / 'worker.json', {'pid': 999999999})
    assert read_status(tmp_path)['state'] == 'error'


def test_save_plan_publishes_matching_id_and_keeps_original(tmp_path):
    from generation import save_plan
    import yaml
    directory = start_generation(tmp_path / 'generations', fake_settings(tmp_path))
    assert wait_job(directory)['state'] == 'done'
    original = (directory / 'plano.yaml').read_bytes()
    source, installed = tmp_path / 'source plans', tmp_path / 'installed plans'
    plan_id, paths = save_plan(directory, source, installed_directory=installed)
    assert len(paths) == 2
    assert plan_id == '7_final_test'
    for path in paths:
        assert path.stem == plan_id
        assert yaml.safe_load(path.read_text())['plan_id'] == plan_id
    assert (directory / 'plano.yaml').read_bytes() == original
    assert save_plan(directory, source, installed_directory=installed) == (plan_id, paths)
    paths[0].write_text('outro plano')
    save_plan(directory, source, installed_directory=installed)
    assert paths[0].read_bytes() == paths[1].read_bytes()
    assert yaml.safe_load(paths[0].read_text())['plan_id'] == '7_final_test'


def test_save_plan_button_uses_remembered_destination(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_TEST_DB', str(tmp_path / 'tests.sqlite3'))
    destination = tmp_path / 'meus planos'
    Store().save_generation_setting('plans_directory', str(destination))
    directory = start_generation(tmp_path / 'generations', fake_settings(tmp_path))
    assert wait_job(directory)['state'] == 'done'
    app = AppTest.from_file(str(APP)).run()
    next(e for e in app.radio if e.label == 'Origem do cenário').set_value('Gerar por seed').run()
    assert app.text_input(key='generation:plans_directory').value == str(destination)
    next(e for e in app.button if e.label == 'Salvar plano na pasta de missões').click().run()
    assert not app.exception
    assert not app.error
    assert len(list(destination.glob('7_final_test.yaml'))) == 1
    assert any('ID para execução' in message.value for message in app.success)


@pytest.mark.parametrize(('mission', 'expected'), [
    ('BMT', '1_basic_manipulation_test'),
    ('BTT1', '2_basic_transportation_test_i'),
    ('BTT2', '3_basic_transportation_test_ii'),
    ('AMT', '4_advanced_transportation_test'),
    ('ATT1', '5_advanced_transportation_test_i'),
    ('ATT2', '6_advanced_transportation_test_ii'),
    ('FINAL', '7_final_test'),
])
def test_save_plan_uses_mission_filename(tmp_path, mission, expected):
    from generation import save_plan, write_json
    directory = tmp_path / 'generation'
    directory.mkdir()
    write_json(directory / 'status.json', {'state': 'done', 'plan': 'plano.yaml'})
    write_json(directory / 'request.json', {'mission': mission})
    (directory / 'plano.yaml').write_text('# Original preservado\nplan_id: plano\nfinish: true\n')
    destination = tmp_path / 'plans'
    destination.mkdir()
    other = destination / 'outra_missao.yaml'
    other.write_text('preservar')
    existing = destination / (expected + '.yaml')
    existing.write_text('plano anterior')
    plan_id, paths = save_plan(directory, destination)
    assert plan_id == expected
    assert paths == [existing]
    assert existing.read_text() == '# Original preservado\nplan_id: ' + expected + '\nfinish: true\n'
    assert other.read_text() == 'preservar'


def test_persistent_planner_pipeline_shares_cache_and_endpoints(tmp_path):
    import json
    import os
    from generation import write_json
    from planner_service import fingerprint, identity
    settings = fake_settings(tmp_path)
    root = tmp_path / 'generations'
    service = root / 'planner'
    service.mkdir(parents=True)
    write_json(service / 'request.json', {'namespace': '/dashboard_test'})
    write_json(service / 'status.json', {'state': 'ready', 'message': 'ok'})
    write_json(service / 'worker.json', {'pid': os.getpid(), 'identity': identity(os.getpid())})
    binary = tmp_path / 'ros2'
    binary.write_text('#!' + sys.executable + '\nimport json, pathlib, sys\n'
                      'args = sys.argv\n'
                      'out = pathlib.Path(args[args.index("--output") + 1])\n'
                      'out.write_text("plan_id: plano\\n")\n'
                      '(out.parent / "args.json").write_text(json.dumps(args))\n')
    planner = {'directory': str(service), 'key': fingerprint(settings)}
    jobs = [start_generation(root, settings, planner=planner) for _ in range(2)]
    for directory in jobs:
        assert wait_job(directory)['state'] == 'done'
    arguments = [json.loads((directory / 'args.json').read_text()) for directory in jobs]
    caches = [args[args.index('--cache') + 1] for args in arguments]
    assert caches[0] == caches[1]
    assert Path(caches[0]).parent == root / 'cache'
    assert '--external-planner' in arguments[0]
    assert arguments[0][arguments[0].index('--planner-server') + 1] == '/dashboard_test/planner_server'
    write_json(service / 'status.json', {'state': 'error', 'message': 'stopped'})
    with pytest.raises(ValueError, match='servidor'):
        start_generation(root, settings, planner=planner)


def test_generation_button_waits_for_ready_server(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_TEST_DB', str(tmp_path / 'tests.sqlite3'))
    state = {'state': 'starting', 'message': 'Iniciando servidor…'}
    monkeypatch.setattr('generation_ui.ensure_service', lambda *a, **kw: state)
    app = AppTest.from_file(str(APP)).run()
    next(e for e in app.radio if e.label == 'Origem do cenário').set_value('Gerar por seed').run()
    assert next(e for e in app.button if e.label == 'Gerar cenário e plano').disabled
    state.update(state='ready', message='Servidor pronto')
    app.run()
    assert not next(e for e in app.button if e.label == 'Gerar cenário e plano').disabled
    assert not app.exception
