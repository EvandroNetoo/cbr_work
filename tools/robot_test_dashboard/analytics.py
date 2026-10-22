"""Agregação apenas de revisões atuais, finalizadas e sem pendências."""
from collections import defaultdict
from statistics import mean, median, pstdev

from domain import INCIDENTS, comparison_key


def eligible_rows(rows):
    return [r for r in rows if r['status'] == 'finalized' and not r['score']['pending']]


def groups(rows):
    result = defaultdict(list)
    for row in rows:
        if row['status'] == 'finalized' and not row['score']['pending']:
            result[comparison_key(row['payload'])].append(row)
    return dict(result)


def summarize(rows):
    eligible = eligible_rows(rows)
    if not eligible:
        return None
    if len({comparison_key(r['payload']) for r in eligible}) != 1:
        raise ValueError('Selecione um único cenário, limite de tempo e conjunto de regras.')
    return _summarize(eligible)


def summarize_mission(rows):
    eligible = eligible_rows(rows)
    if not eligible:
        return None
    if len({r['payload']['scenario']['mission'] for r in eligible}) != 1:
        raise ValueError('Selecione uma única missão para calcular os indicadores.')
    return _summarize(eligible)


def _summarize(eligible):
    scores = [r['score']['total'] for r in eligible]
    completed = [r['score']['elapsed_s'] for r in eligible if r['score']['completed']]
    losses = defaultdict(float)
    for row in eligible:
        for loss in row['score']['losses']:
            losses[(loss['area'], loss['object'], loss['cause'])] += loss['points']
    return {'count': len(scores), 'mean': mean(scores), 'median': median(scores),
            'best': max(scores), 'stddev': pstdev(scores),
            'completion_rate': sum(r['score']['completed'] for r in eligible) / len(eligible),
            'perfect_rate': sum(r['score']['perfect'] for r in eligible) / len(eligible),
            'mean_completed_s': mean(completed) if completed else None,
            'losses': [{'area': a, 'object': o, 'cause': c, 'points': value}
                       for (a, o, c), value in sorted(losses.items(), key=lambda x: -x[1])],
            'timeline': [{'data': r['payload']['tested_at'], 'pontos': r['score']['total'],
                          'tempo_s': r['score']['elapsed_s'], 'teste': r['record_id'],
                          'seed': r['payload']['seed']}
                         for r in sorted(eligible, key=lambda r: r['payload']['tested_at'])]}


def accumulated_errors(rows):
    """Global incident counts include restarts; point losses use the scored attempt."""
    eligible = eligible_rows(rows)
    incidents = defaultdict(lambda: {'count': 0, 'tests': set()})
    losses = defaultdict(lambda: {'count': 0, 'points': 0, 'tests': set()})
    for row in eligible:
        for attempt in row['payload']['attempts']:
            for incident in attempt['incidents']:
                item = incidents[incident['type']]
                item['count'] += 1
                item['tests'].add(row['record_id'])
        for loss in row['score']['losses']:
            item = losses[(loss['kind'], loss['cause'])]
            item['count'] += 1
            item['points'] += loss['points']
            item['tests'].add(row['record_id'])
    return {
        'count': len(eligible),
        'incidents': [{'Tipo': INCIDENTS.get(kind, kind), 'Ocorrências': item['count'],
                       'Testes afetados': len(item['tests'])}
                      for kind, item in sorted(incidents.items(), key=lambda pair: -pair[1]['count'])],
        'losses': [{'Tipo': kind, 'Causa': cause, 'Registros de perda': item['count'],
                    'Testes afetados': len(item['tests']), 'Pontos perdidos': item['points']}
                   for (kind, cause), item in sorted(losses.items(), key=lambda pair: -pair[1]['points'])],
    }
