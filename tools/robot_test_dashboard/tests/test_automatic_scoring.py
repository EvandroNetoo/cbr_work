from copy import deepcopy
import pytest

from domain import add_incident, new_record, restart, score, upgrade_rules
from storage import Store


@pytest.mark.parametrize('followed_line,bonus,total', [(False, 660, 3960), (True, 0, 3300)])
def test_line_bonus(scenario, followed_line, bonus, total):
    p = new_record(scenario, '8')
    p['attempts'][0]['followed_line'] = followed_line
    s = score(p)
    assert s['navigation_bonus'] == bonus
    assert s['total'] == total
    assert not s['pending']


@pytest.mark.parametrize('count,rate', [(0, 0), (1, .05), (2, .1), (3, .25), (4, .25)])
def test_tape_automatic_non_cumulative(scenario, count, rate):
    p = new_record(scenario, '8')
    for _ in range(count):
        add_incident(p['attempts'][0], 'fita')
    s = score(p)
    assert s['tape_count'] == count
    assert s['tape_rate'] == rate
    assert s['tape_loss'] == 3960 * rate
    assert s['total'] == 3960 * (1-rate)
    assert not s['pending']
    assert not p['attempts'][0]['adjustments']


def test_bonus_tape_fixed_penalty_and_severe_order(scenario):
    p = new_record(scenario, '8')
    a = p['attempts'][0]
    a['objects'][0]['captured'] = False
    add_incident(a, 'queda')
    add_incident(a, 'fita')
    add_incident(a, 'colisao_grave', decision='encerrar')
    s = score(p)
    assert s['navigation_bonus'] == 620
    assert s['tape_loss'] == 186
    assert s['total'] == (3100 * 1.2 * .95) / 2


def test_perfect_and_time_included_in_bonus(scenario):
    p = new_record(scenario, '8')
    p['attempts'][0].update(completed=True, elapsed_s=1200)
    s = score(p)
    assert s['perfect_bonus'] == 300
    assert s['time_bonus'] == 600
    assert s['navigation_bonus'] == 840
    assert s['total'] == 5040


def test_restart_discards_first_tape_and_navigation(scenario):
    p = new_record(scenario, '8')
    add_incident(p['attempts'][0], 'fita')
    p['attempts'][0]['elapsed_s'] = 60
    restart(p)
    p['attempts'][1]['followed_line'] = True
    s = score(p)
    assert s['elapsed_s'] == 60
    assert s['tape_count'] == s['navigation_bonus'] == 0
    assert s['total'] == 3300


def test_upgrade_preserves_old_revision_and_removes_automatic_manual_adjustments(tmp_path, record):
    store = Store(tmp_path / 'history.sqlite3')
    add_incident(record['attempts'][0], 'fita')
    record['attempts'][0]['adjustments'].append({'id': 'nav', 'label': 'Bônus de navegação sem marcações', 'resolved': True, 'points': 660, 'reason': '20%'})
    old = store.save(record)
    upgraded = deepcopy(record)
    upgrade_rules(upgraded)
    new = store.save(upgraded, expected_revision=1)
    assert old['score']['pending']
    assert not new['score']['pending']
    assert new['score']['total'] == 3762
    assert store.history(record['id'])[-1] == old
    restored = Store(tmp_path / 'restored.sqlite3')
    assert restored.restore(store.export_json()) == 2
    assert restored.history(record['id']) == store.history(record['id'])
