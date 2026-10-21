from copy import deepcopy
from pathlib import Path

from streamlit.testing.v1 import AppTest

from storage import Store
from domain import new_record


APP = Path(__file__).resolve().parents[1] / 'app.py'


def start_app(tmp_path, monkeypatch, record=None):
    monkeypatch.setenv('ROBOT_TEST_DB', str(tmp_path / 'ui.sqlite3'))
    store = Store()
    app = AppTest.from_file(str(APP), default_timeout=20)
    if record:
        row = store.save(record)
        app.session_state['work'] = deepcopy(record)
        app.session_state['work_revision'] = row['revision']
        app.session_state['work_status'] = row['status']
        app.session_state['editor_epoch'] = 1
    app.run()
    assert not app.exception
    assert not app.error
    return app, store


def by_label(elements, text):
    return next(e for e in elements if e.label == text)


def test_empty_pages(tmp_path, monkeypatch):
    app, _ = start_app(tmp_path, monkeypatch)
    app.radio[0].set_value('Histórico').run()
    assert not app.exception
    app.radio[0].set_value('Estatísticas').run()
    assert not app.exception
    assert 'Finalize' in app.info[0].value


def test_edit_autosave_reopen_review_and_finalize(tmp_path, monkeypatch, record):
    app, store = start_app(tmp_path, monkeypatch, record)
    prefix = record['id'] + ':1:1'
    app.checkbox(key=f"{prefix}:captured:{record['scenario']['objects'][0]['id']}").uncheck().run()
    assert not app.exception
    assert not app.error
    latest = store.latest(record['id'])
    assert latest['score']['missing'] == 100
    assert latest['status'] == 'draft'
    assert by_label(app.checkbox, 'Missão concluída (todas as tarefas)').disabled
    reopened = AppTest.from_file(str(APP), default_timeout=20).run()
    reopened.radio[0].set_value('Histórico').run()
    assert not reopened.exception
    by_label(reopened.button, 'Abrir rascunho atual para edição').click().run()
    assert not reopened.exception
    assert not reopened.error
    by_label(reopened.checkbox, 'Revisei as tarefas, o tempo, os incidentes e a memória de cálculo').check().run()
    by_label(reopened.button, 'Finalizar teste').click().run()
    assert not reopened.exception
    assert not reopened.error
    assert store.latest(record['id'])['status'] == 'finalized'


def test_bulk_unperformed_and_restart(tmp_path, monkeypatch, record):
    app, store = start_app(tmp_path, monkeypatch, record)
    app.multiselect[0].set_value(['Chegada · WS_2', 'Capturar · objeto 4']).run()
    by_label(app.button, 'Marcar selecionadas como não realizadas').click().run()
    assert not app.exception
    assert not app.error
    assert store.latest(record['id'])['score']['missing'] == 200
    by_label(app.number_input, 'Tempo consumido nesta tentativa (segundos) · tentativa 1').set_value(120).run()
    by_label(app.checkbox, 'Confirmo o reinício da rodada').check().run()
    by_label(app.button, 'Reiniciar rodada').click().run()
    assert not app.exception
    assert not app.error
    latest = store.latest(record['id'])
    assert len(latest['payload']['attempts']) == 2
    assert latest['score']['elapsed_s'] == 120
    assert latest['score']['missing'] == 0


def test_incident_form_grave_restart_and_pending_adjustment(tmp_path, monkeypatch, record):
    app, store = start_app(tmp_path, monkeypatch, record)
    by_label(app.selectbox, 'Tipo de incidente').set_value('colisao_grave')
    by_label(app.selectbox, 'Decisão para colisão grave').set_value('reiniciar')
    by_label(app.button, 'Adicionar incidente').click().run()
    assert not app.exception
    assert not app.error
    assert len(store.latest(record['id'])['payload']['attempts']) == 2
    by_label(app.selectbox, 'Tipo de incidente').set_value('fita')
    by_label(app.button, 'Adicionar incidente').click().run()
    assert not app.exception
    assert not app.error
    assert store.latest(record['id'])['score']['pending'] == ['Colisão com fita']
    by_label(app.number_input, 'Valor em pontos (+ / −)').set_value(-165)
    by_label(app.text_area, 'Justificativa e base do cálculo').input('5% sobre 3300 pontos')
    by_label(app.checkbox, 'Decisão resolvida').check().run()
    assert not app.exception
    assert not app.error
    assert store.latest(record['id'])['score']['total'] == 3135
    assert not store.latest(record['id'])['score']['pending']


def test_history_correction_void_and_statistics(tmp_path, monkeypatch, record):
    app, store = start_app(tmp_path, monkeypatch, record)
    by_label(app.checkbox, 'Missão concluída (todas as tarefas)').check().run()
    by_label(app.checkbox, 'Revisei as tarefas, o tempo, os incidentes e a memória de cálculo').check().run()
    by_label(app.button, 'Finalizar teste').click().run()
    app.radio[0].set_value('Estatísticas').run()
    assert not app.exception
    assert not app.error
    assert len(app.metric) == 7
    app.radio[0].set_value('Histórico').run()
    by_label(app.text_input, 'Justificativa para corrigir ou anular o registro').input('Correção da observação').run()
    by_label(app.button, 'Criar revisão para correção').click().run()
    assert not app.exception
    assert not app.error
    assert store.latest(record['id'])['status'] == 'draft'
    by_label(app.checkbox, 'Revisei as tarefas, o tempo, os incidentes e a memória de cálculo').check().run()
    by_label(app.button, 'Finalizar teste').click().run()
    by_label(app.text_input, 'Justificativa para corrigir ou anular o registro').input('Teste duplicado').run()
    by_label(app.button, 'Anular registro').click().run()
    assert not app.exception
    assert not app.error
    assert store.latest(record['id'])['status'] == 'void'


def test_incident_requires_new_review(tmp_path, monkeypatch, record):
    app, store = start_app(tmp_path, monkeypatch, record)
    by_label(app.checkbox, 'Revisei as tarefas, o tempo, os incidentes e a memória de cálculo').check().run()
    assert not by_label(app.button, 'Finalizar teste').disabled
    by_label(app.selectbox, 'Tipo de incidente').set_value('queda')
    by_label(app.button, 'Adicionar incidente').click().run()
    assert not app.exception
    assert not app.error
    assert by_label(app.button, 'Finalizar teste').disabled
    assert store.latest(record['id'])['score']['penalties'] == 100


def test_automatic_line_and_tape_without_manual_arbitration(tmp_path, monkeypatch, scenario):
    record = new_record(scenario, '8')
    app, store = start_app(tmp_path, monkeypatch, record)
    assert store.latest(record['id'])['score']['total'] == 3960
    assert not any(e.label == 'Novo ajuste de arbitragem' for e in app.selectbox)
    assert not any(e.label == 'Decisão para colisão grave' for e in app.selectbox)
    by_label(app.checkbox, 'Usou seguidor de linha').check().run()
    assert store.latest(record['id'])['score']['total'] == 3300
    by_label(app.selectbox, 'Tipo de incidente').set_value('fita').run()
    by_label(app.button, 'Adicionar incidente').click().run()
    assert not app.exception
    assert not app.error
    latest = store.latest(record['id'])
    assert latest['score']['total'] == 3135
    assert not latest['score']['pending']
    assert not any(e.label == 'Valor em pontos (+ / −)' for e in app.number_input)
    by_label(app.checkbox, 'Usou seguidor de linha').uncheck().run()
    assert store.latest(record['id'])['score']['total'] == 3762


def test_upgrade_button_keeps_old_revision(tmp_path, monkeypatch, record):
    app, store = start_app(tmp_path, monkeypatch, record)
    by_label(app.button, 'Atualizar ficha para cálculo automático').click().run()
    assert not app.exception
    assert not app.error
    assert store.latest(record['id'])['score']['total'] == 3960
    assert store.history(record['id'])[-1]['score']['total'] == 3300
