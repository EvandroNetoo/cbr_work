from copy import deepcopy
from pathlib import Path

import pytest
from streamlit.testing.v1 import AppTest
import yaml

from analytics import accumulated_errors, summarize, summarize_mission
from domain import INCIDENTS, add_incident, import_scenario, new_record, restart, score
from storage import Store


def mission_record(scenario, mission='FINAL', seed='8'):
    raw = deepcopy(scenario['raw'])
    raw['task_id'] = mission
    imported = import_scenario(yaml.safe_dump(raw), f'{mission}_seed{seed}_equipe.yaml')
    return new_record(imported, seed)


def row(payload, status='finalized'):
    return {'record_id': payload['id'], 'payload': payload, 'score': score(payload), 'status': status}


def test_mission_summary_combines_seeds_but_rejects_other_missions(scenario):
    first = mission_record(scenario, seed='8')
    second = mission_record(scenario, seed='9')
    add_incident(second['attempts'][0], 'queda')
    rows = [row(first), row(second)]
    result = summarize_mission(rows)
    assert result['count'] == 2
    assert result['mean'] == sum(item['score']['total'] for item in rows) / 2
    assert {item['seed'] for item in result['timeline']} == {'8', '9'}
    with pytest.raises(ValueError, match='único cenário'):
        summarize(rows)
    with pytest.raises(ValueError, match='única missão'):
        summarize_mission(rows + [row(mission_record(scenario, 'ATT2'))])
    assert summarize_mission([row(first, 'draft')]) is None


def test_global_errors_combine_missions_include_restarts_and_exclude_ineligible(scenario):
    final = mission_record(scenario)
    add_incident(final['attempts'][0], 'queda')
    restart(final)
    add_incident(final['attempts'][1], 'queda')
    att2 = mission_record(scenario, 'ATT2')
    add_incident(att2['attempts'][0], 'queda')
    draft = row(att2, 'draft')
    pending = deepcopy(row(att2))
    pending['score']['pending'] = ['Revisão pendente']
    result = accumulated_errors([row(final), row(att2), draft, pending, row(att2, 'void')])
    assert result['count'] == 2
    assert result['incidents'] == [{'Tipo': INCIDENTS['queda'], 'Ocorrências': 3, 'Testes afetados': 2}]
    assert result['losses'][0]['Registros de perda'] == 2
    assert result['losses'][0]['Pontos perdidos'] == 200
    assert result['losses'][0]['Testes afetados'] == 2


def test_statistics_ui_keeps_results_separate_and_errors_global(tmp_path, monkeypatch, scenario):
    monkeypatch.setenv('ROBOT_TEST_DB', str(tmp_path / 'stats.sqlite3'))
    store = Store()
    final = mission_record(scenario)
    add_incident(final['attempts'][0], 'queda')
    att2 = mission_record(scenario, 'ATT2')
    add_incident(att2['attempts'][0], 'queda')
    add_incident(att2['attempts'][0], 'queda')
    final_row = store.save(final, status='finalized', reviewed=True)
    att2_row = store.save(att2, status='finalized', reviewed=True)
    app = AppTest.from_file(str(Path(__file__).resolve().parents[1] / 'app.py')).run()
    next(e for e in app.radio if e.label == 'Navegação').set_value('Estatísticas').run()
    assert not app.exception
    assert not app.error
    for mission, expected in [('FINAL', final_row), ('ATT2', att2_row)]:
        app.selectbox(key='statistics:mission').set_value(mission).run()
        assert not app.exception
        assert next(e for e in app.metric if e.label == 'Média').value == f"{expected['score']['total']:.1f}"
        assert next(e for e in app.metric if e.label == 'Testes finalizados').value == '1'
        global_incidents = next(e.value for e in app.dataframe if 'Ocorrências' in e.value.columns)
        assert global_incidents['Ocorrências'].sum() == 3
    app.selectbox(key='statistics:scenario:ATT2').set_value(0).run()
    assert not app.exception
    assert not app.error
