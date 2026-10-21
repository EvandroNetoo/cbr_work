"""Agregação apenas de revisões atuais, finalizadas e sem pendências."""
from collections import defaultdict
from statistics import mean, median, pstdev

from domain import comparison_key


def groups(rows):
    result = defaultdict(list)
    for row in rows:
        if row['status'] == 'finalized' and not row['score']['pending']:
            result[comparison_key(row['payload'])].append(row)
    return dict(result)


def summarize(rows):
    eligible = [r for r in rows if r['status'] == 'finalized' and not r['score']['pending']]
    if not eligible:
        return None
    if len({comparison_key(r['payload']) for r in eligible}) != 1:
        raise ValueError('Selecione um único cenário, limite de tempo e conjunto de regras.')
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
                          'tempo_s': r['score']['elapsed_s'], 'teste': r['record_id']}
                         for r in sorted(eligible, key=lambda r: r['payload']['tested_at'])]}
