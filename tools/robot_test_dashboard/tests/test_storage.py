from copy import deepcopy
import json
import sqlite3

import pytest

from analytics import groups, summarize
from domain import new_record
from storage import Store


def test_revisions_and_reopen(tmp_path, record):
    store = Store(tmp_path / 'tests.sqlite3')
    row = store.save(record)
    assert row['status'] == 'draft'
    assert not groups(store.list_latest())
    record['attempts'][0]['completed'] = True
    store.save(record, status='finalized', expected_revision=1, reviewed=True)
    reopened = Store(store.path)
    assert reopened.latest(record['id'])['score']['perfect']
    assert reopened.history(record['id'])[-1]['payload']['attempts'][0]['completed'] is False
    assert summarize(reopened.list_latest())['count'] == 1
    with pytest.raises(ValueError, match='justificativa'):
        reopened.save(record, expected_revision=2)
    reopened.save(record, expected_revision=2, reason='Corrigir observação')
    assert len(reopened.history(record['id'])) == 3
    assert not groups(reopened.list_latest())


def test_review_void_and_concurrency(tmp_path, record):
    store = Store(tmp_path / 'tests.sqlite3')
    store.save(record)
    with pytest.raises(ValueError, match='Revise'):
        store.save(record, status='finalized', expected_revision=1)
    with pytest.raises(ValueError, match='outra sessão'):
        store.save(record, expected_revision=0)
    store.save(record, status='finalized', expected_revision=1, reviewed=True)
    store.save(record, status='void', expected_revision=2, reason='Teste duplicado')
    assert not groups(store.list_latest())
    assert store.latest(record['id'])['reason'] == 'Teste duplicado'


def test_export_restore_all_revisions_files_and_scores(tmp_path, record):
    source = Store(tmp_path / 'source' / 'tests.sqlite3')
    source.save(record)
    record['notes'] = 'Observação de revisão'
    source.save(record, status='finalized', expected_revision=1, reviewed=True)
    target = Store(tmp_path / 'target' / 'tests.sqlite3')
    assert target.restore(source.export_json()) == 2
    assert target.history(record['id']) == source.history(record['id'])
    assert target.latest(record['id'])['payload']['plan']['text'] == 'plan_id: exemplo\n'
    assert target.restore(source.export_json()) == 0
    assert len(list((target.path.parent / 'backups').glob('*.sqlite3'))) == 2
    csvs = target.csv_exports()
    assert set(csvs) == {'revisoes.csv', 'tarefas.csv', 'incidentes.csv'}
    assert 'Observação de revisão' in csvs['revisoes.csv']


def test_restore_conflict_rolls_back_and_corruption_rejected(tmp_path, record, scenario):
    store = Store(tmp_path / 'tests.sqlite3')
    store.save(record)
    exported = json.loads(store.export_json())
    conflict = deepcopy(exported['revisions'][0])
    conflict['reason'] = 'Conflito'
    extra = deepcopy(conflict)
    extra['record_id'] = extra['payload']['id'] = 'novo-id'
    exported['revisions'] = [extra, conflict]
    with pytest.raises(ValueError, match='Conflito'):
        store.restore(json.dumps(exported))
    assert store.latest('novo-id') is None
    exported['revisions'][0]['score']['total'] += 1
    with pytest.raises(ValueError, match='inconsistente'):
        store.restore(json.dumps(exported))


def test_backup_daily_consistent(tmp_path, record):
    store = Store(tmp_path / 'tests.sqlite3')
    store.save(record)
    backup = store.backup()
    assert store.backup() == backup
    with sqlite3.connect(backup) as db:
        assert db.execute('SELECT count(*) FROM revisions').fetchone()[0] == 1


def test_comparison_separates_seed_content_time_rules_and_pending(tmp_path, scenario):
    store = Store(tmp_path / 'tests.sqlite3')
    for n in range(6):
        p = new_record(scenario, str(8 if n != 1 else 9))
        if n == 2:
            p['limit_s'] = 1200
        if n == 3:
            p['rules']['arrival_policy'] = 'Outra interpretação'
        if n == 4:
            p['attempts'][0]['adjustments'].append({'id': 'x', 'label': 'Pendente', 'points': 0, 'resolved': False, 'reason': ''})
        store.save(p, status='finalized', reviewed=True)
    sets = groups(store.list_latest())
    assert len(sets) == 4
    assert sorted(len(s) for s in sets.values()) == [1, 1, 1, 2]
    main = next(g for g in sets.values() if len(g) == 2)
    summary = summarize(main)
    assert summary['count'] == 2
    assert summary['mean'] == summary['median'] == summary['best'] == 3960
    assert summary['stddev'] == 0
    assert summary['completion_rate'] == summary['perfect_rate'] == 0
    assert summary['mean_completed_s'] is None
    with pytest.raises(ValueError, match='único cenário'):
        summarize(store.list_latest())
