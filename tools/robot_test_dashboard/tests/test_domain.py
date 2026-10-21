import pytest

from domain import RULES, add_incident, import_scenario, restart, score


def test_commander_final_acceptance(scenario, record):
    assert scenario['mission'] == 'FINAL'
    assert scenario['seed'] == '8'
    assert len(scenario['objects']) == 10
    assert len(scenario['areas']) == 8
    assert record['limit_s'] == 1800
    record['attempts'][0]['completed'] = True
    record['attempts'][0]['elapsed_s'] = 1800
    result = score(record)
    assert result['base'] == 3300
    assert result['perfect_bonus'] == 300
    assert result['time_bonus'] == 0
    assert result['total'] == 3600


@pytest.mark.parametrize('elapsed,completed,perfect,time', [
    (1200, True, True, 600), (1800, True, True, 0),
    (1801, True, False, 0), (1200, False, False, 0),
])
def test_time_boundaries(record, elapsed, completed, perfect, time):
    record['attempts'][0].update(elapsed_s=elapsed, completed=completed)
    result = score(record)
    assert result['perfect'] == perfect
    assert result['time_bonus'] == time


def test_independent_outcomes_and_drop(record):
    attempt = record['attempts'][0]
    attempt['objects'][0]['captured'] = False
    attempt['objects'][1]['delivered'] = False
    add_incident(attempt, 'queda', obj=attempt['objects'][2]['id'])
    result = score(record)
    assert result['missing'] == 200
    assert result['penalties'] == 100
    assert result['total'] == 3000
    assert attempt['objects'][2]['captured'] is True
    assert attempt['objects'][0]['identified'] is True


def test_arrivals_once_per_distinct_area(record):
    record['attempts'][0]['arrivals']['WS_2'] = False
    assert score(record)['missing'] == 100
    assert score(record)['base'] == 3300


def test_deduction_per_process_and_repeated_incidents(record):
    attempt = record['attempts'][0]
    for process in ('pick4_WS2', 'pick4_WS2', 'place4_ST'):
        add_incident(attempt, 'manipulacao_deduzida', process=process)
    for _ in range(2):
        add_incident(attempt, 'colisao_leve')
        add_incident(attempt, 'intervencao_humana')
    assert score(record)['penalties'] == 400


def test_restart_preserves_first_attempt_and_time(record):
    first = record['attempts'][0]
    first['elapsed_s'] = 300
    first['objects'][0]['captured'] = False
    add_incident(first, 'queda')
    restart(record)
    second = record['attempts'][1]
    second.update(completed=True, elapsed_s=600)
    result = score(record)
    assert result['elapsed_s'] == 900
    assert result['time_bonus'] == 900
    assert result['penalties'] == 0
    assert result['missing'] == 0
    assert first['incidents'][0]['type'] == 'queda'
    with pytest.raises(ValueError, match='um reinício'):
        restart(record)


def test_severe_collision_end_and_restart(record):
    first = record['attempts'][0]
    incident = add_incident(first, 'colisao_grave', decision='encerrar')
    assert score(record)['total'] == 1650
    assert score(record)['time_bonus'] == 0
    with pytest.raises(ValueError, match='encerramento'):
        restart(record)
    incident['decision'] = 'reiniciar'
    with pytest.raises(ValueError, match='efetue o reinício'):
        score(record)
    restart(record)
    add_incident(record['attempts'][1], 'colisao_grave', decision='encerrar')
    assert score(record)['total'] == 1650


def test_pending_adjudication_and_signed_adjustments(record):
    attempt = record['attempts'][0]
    add_incident(attempt, 'fita')
    assert score(record)['pending'] == ['Colisão com fita']
    adj = attempt['adjustments'][0]
    adj.update(resolved=True, points=-165, reason='Uma colisão: 5% sobre 3300 pontos.')
    assert score(record)['total'] == 3135
    assert not score(record)['pending']
    adj['reason'] = ''
    with pytest.raises(ValueError, match='justificativa'):
        score(record)


def test_disqualification_cannot_restart(record):
    add_incident(record['attempts'][0], 'desclassificacao')
    assert score(record)['total'] == 0
    with pytest.raises(ValueError, match='desclassificação'):
        restart(record)


def test_snapshot_rules_and_scenario(record, scenario):
    scenario['objects'][0]['source'] = 'EDITADO'
    assert record['scenario']['objects'][0]['source'] != 'EDITADO'
    record['rules']['arrival'] = 42
    assert RULES['arrival'] == 100
    assert score(record)['base'] == 2836


def test_hash_ignores_comments_but_not_contents(scenario):
    source = scenario['file']
    assert import_scenario(source['text'] + '\n# outro comentário\n', source['name'])['hash'] == scenario['hash']
    assert import_scenario(source['text'].replace('duration_min: 30', 'duration_min: 20'), source['name'])['hash'] != scenario['hash']


@pytest.mark.parametrize('mutation', [
    lambda p: p.update(limit_s=float('nan')),
    lambda p: p['attempts'][0].update(elapsed_s=-1),
    lambda p: p['attempts'][0]['objects'].pop(),
    lambda p: p['attempts'][0]['objects'][0].update(id=999),
    lambda p: p['scenario'].update(hash='changed'),
])
def test_invalid_data_rejected(record, mutation):
    mutation(record)
    with pytest.raises(ValueError):
        score(record)


def test_completed_requires_all_tasks(record):
    record['attempts'][0]['completed'] = True
    record['attempts'][0]['objects'][0]['delivered'] = False
    with pytest.raises(ValueError, match='todas as tarefas'):
        score(record)


def test_invalid_yaml_and_incomplete_scenario(scenario):
    with pytest.raises(ValueError):
        import_scenario('objects: [', 'bad.yaml')
    with pytest.raises(ValueError):
        import_scenario('task_id: UNKNOWN', 'bad.yaml')
    with pytest.raises(ValueError, match='todos os objetos'):
        import_scenario(scenario['file']['text'].replace('obj_ids: [15, 13]', 'obj_ids: [15]'), 'bad.yaml')
