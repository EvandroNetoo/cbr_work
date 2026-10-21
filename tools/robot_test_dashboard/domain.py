"""Snapshots de missões e pontuação. Não importa ROS nem Streamlit."""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from uuid import uuid4
from zoneinfo import ZoneInfo

import yaml


TZ = ZoneInfo('America/Sao_Paulo')
RULES = {
    'version': 'CBR-2026-local-v1',
    'source': 'CBR_RoboCup_Work_Rulebook_2026.pdf, seções 5.5 e 6',
    'arrival_policy': 'Uma chegada por área distinta ativa; sem start/finish.',
    'arrival': 100, 'identified': 50, 'captured': 100, 'delivered': 100,
    'perfect_per_object': 30, 'time_per_second': 1,
    'penalties': {'queda': 100, 'manipulacao_incorreta': 50,
                  'manipulacao_deduzida': 50, 'colisao_leve': 50,
                  'intervencao_humana': 100},
    'interpretations': [
        'Bônus de perfeição: todas as tarefas concluídas dentro do limite; incidentes são descontados separadamente.',
        'Manipulação deduzida: uma cobrança por processo identificado, em cada tentativa.',
        'Tempo informado por tentativa; tempo anterior ao reinício continua consumido.',
        'Ajustes de arbitragem são valores em pontos assinados, aplicados após bônus e penalidades fixas.',
        'Colisão grave com encerramento: metade do subtotal após ajustes; sem bônus de conclusão.',
        'Percentuais, simplificações e bônus sem marcações exigem valor e justificativa explícitos.',
        'Sem piso ou arredondamento implícito; valores negativos são preservados.',
    ],
}
INCIDENTS = {
    'queda': 'Queda de objeto', 'manipulacao_incorreta': 'Manipulação incorreta',
    'manipulacao_deduzida': 'Manipulação deduzida', 'colisao_leve': 'Colisão leve',
    'colisao_grave': 'Colisão grave', 'fita': 'Colisão com fita',
    'intervencao_humana': 'Intervenção humana', 'desclassificacao': 'Desclassificação',
}
MISSIONS = {'BMT', 'BTT1', 'BTT2', 'AMT', 'ATT1', 'ATT2', 'FINAL'}


def now():
    return datetime.now(TZ).isoformat(timespec='microseconds')


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def number(value, label, minimum=0):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
        raise ValueError(f'{label}: informe um número finito maior ou igual a {minimum}.')
    return value


def import_scenario(text, filename):
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f'YAML inválido: {exc}') from exc
    if not isinstance(raw, dict) or raw.get('task_id') not in MISSIONS:
        raise ValueError('Selecione o YAML de equipe do Commander (task_id inválido).')
    duration = number(raw.get('duration_min'), 'duration_min', 0.01)
    areas = raw.get('active_service_areas')
    if not isinstance(areas, list) or not areas or any(not isinstance(x, str) or not x.strip() for x in areas) or len(set(areas)) != len(areas):
        raise ValueError('active_service_areas deve listar áreas únicas.')
    objects = raw.get('objects')
    if not isinstance(objects, list) or not objects or any(not isinstance(o, dict) or type(o.get('id')) is not int for o in objects):
        raise ValueError('objects deve listar objetos com IDs inteiros.')
    ids = [o['id'] for o in objects]
    if len(set(ids)) != len(ids):
        raise ValueError('IDs de objetos duplicados.')
    locations = {}
    for state in ('start_state', 'finish_state'):
        entries = raw.get(state)
        if not isinstance(entries, dict) or set(entries) != set(areas):
            raise ValueError(f'{state} deve conter exatamente as áreas ativas.')
        found = {}
        for area, entry in entries.items():
            if not isinstance(entry, dict) or not isinstance(entry.get('obj_ids'), list):
                raise ValueError(f'{state}.{area}: obj_ids inválido.')
            for obj in entry['obj_ids']:
                if type(obj) is not int or obj not in ids or obj in found:
                    raise ValueError(f'{state}: objeto desconhecido ou duplicado: {obj}.')
                found[obj] = area
        if set(found) != set(ids):
            raise ValueError(f'{state}: todos os objetos devem ter uma localização.')
        locations[state] = found
    seed = re.search(r'seed[_-]?(\d+)', Path(filename).stem, re.IGNORECASE)
    # O hash usa o conteúdo semântico, preservando ordem relevante dos objetos.
    digest = hashlib.sha256(canonical(raw).encode()).hexdigest()
    return {
        'mission': raw['task_id'], 'seed': seed.group(1) if seed else '',
        'duration_s': duration * 60, 'areas': areas,
        'objects': [{'id': i, 'source': locations['start_state'][i],
                     'destination': locations['finish_state'][i]} for i in ids],
        'hash': digest, 'raw': raw,
        'file': {'name': Path(filename).name, 'text': text},
    }


def new_attempt(scenario):
    return {'arrivals': {a: True for a in scenario['areas']},
            'objects': [{**o, 'identified': True, 'captured': True, 'delivered': True}
                        for o in scenario['objects']],
            'elapsed_s': 0, 'completed': False, 'incidents': [], 'adjustments': []}


def new_record(scenario, seed, plan=None):
    if not str(seed).strip():
        raise ValueError('Confirme a seed antes de criar o teste.')
    return {'id': str(uuid4()), 'created_at': now(), 'tested_at': now(),
            'scenario': deepcopy(scenario), 'seed': str(seed).strip(),
            'limit_s': scenario['duration_s'], 'plan': deepcopy(plan),
            'rules': deepcopy(RULES), 'attempts': [new_attempt(scenario)], 'notes': ''}


def restart(record):
    if len(record['attempts']) != 1:
        raise ValueError('Só é permitido um reinício por rodada.')
    first = record['attempts'][0]
    if first['completed']:
        raise ValueError('Marque a tentativa anterior como interrompida antes de reiniciar.')
    if any(i['type'] == 'desclassificacao' for i in first['incidents']):
        raise ValueError('Uma desclassificação não permite reinício.')
    if any(i['type'] == 'colisao_grave' and i.get('decision') != 'reiniciar' for i in first['incidents']):
        raise ValueError('A colisão grave foi registrada com encerramento.')
    record['attempts'].append(new_attempt(record['scenario']))


def add_incident(attempt, kind, *, area='', obj=None, cause='', process='', decision=''):
    if kind not in INCIDENTS:
        raise ValueError('Incidente desconhecido.')
    if kind == 'manipulacao_deduzida' and not process.strip():
        raise ValueError('Identifique o processo de captura/depósito para evitar cobrança duplicada.')
    item = {'id': str(uuid4()), 'type': kind, 'area': area, 'object': obj,
            'cause': cause, 'process': process.strip(), 'decision': decision}
    attempt['incidents'].append(item)
    if kind == 'fita':
        attempt['adjustments'].append({'id': item['id'], 'label': 'Colisão com fita',
                                       'resolved': False, 'points': 0, 'reason': ''})
    return item


def validate(record):
    scenario = record['scenario']
    # Validação também protege a restauração de dados incompatíveis/corrompidos.
    imported = import_scenario(scenario['file']['text'], scenario['file']['name'])
    for key in ('hash', 'mission', 'areas', 'objects', 'duration_s', 'raw'):
        if imported[key] != scenario[key]:
            raise ValueError(f'Cenário inconsistente: {key}.')
    if not str(record['seed']).strip():
        raise ValueError('Seed obrigatória.')
    number(record['limit_s'], 'Tempo limite', 1)
    datetime.fromisoformat(record['tested_at'])
    rules = record['rules']
    for key in ('arrival', 'identified', 'captured', 'delivered', 'perfect_per_object', 'time_per_second'):
        number(rules[key], f'Regra {key}')
    for value in rules['penalties'].values():
        number(value, 'Penalidade')
    attempts = record['attempts']
    if not 1 <= len(attempts) <= 2:
        raise ValueError('A rodada deve ter uma ou duas tentativas.')
    for index, attempt in enumerate(attempts):
        number(attempt['elapsed_s'], 'Tempo consumido')
        if type(attempt['completed']) is not bool:
            raise ValueError('Conclusão inválida.')
        if index < len(attempts) - 1 and attempt['completed']:
            raise ValueError('Uma tentativa reiniciada não pode estar concluída.')
        if set(attempt['arrivals']) != set(scenario['areas']) or any(type(v) is not bool for v in attempt['arrivals'].values()):
            raise ValueError('Ficha de chegadas inconsistente.')
        if len(attempt['objects']) != len(scenario['objects']):
            raise ValueError('Ficha de objetos incompleta.')
        for obj, expected in zip(attempt['objects'], scenario['objects']):
            if any(obj.get(k) != expected[k] for k in ('id', 'source', 'destination')) or any(type(obj.get(k)) is not bool for k in ('identified', 'captured', 'delivered')):
                raise ValueError('Ficha de objetos inconsistente.')
        if attempt['completed'] and (not all(attempt['arrivals'].values()) or not all(o[k] for o in attempt['objects'] for k in ('identified', 'captured', 'delivered'))):
            raise ValueError('Uma missão concluída deve ter todas as tarefas realizadas.')
        incident_ids = set()
        severe = []
        for incident in attempt['incidents']:
            if incident['id'] in incident_ids or incident['type'] not in INCIDENTS:
                raise ValueError('Incidente inválido ou duplicado.')
            incident_ids.add(incident['id'])
            if incident.get('area') and incident['area'] not in scenario['areas']:
                raise ValueError('Área do incidente desconhecida.')
            if incident.get('object') is not None and incident['object'] not in [o['id'] for o in scenario['objects']]:
                raise ValueError('Objeto do incidente desconhecido.')
            if incident['type'] == 'manipulacao_deduzida' and not incident.get('process', '').strip():
                raise ValueError('Processo de manipulação obrigatório.')
            if incident['type'] == 'colisao_grave':
                severe.append(incident)
                if incident.get('decision') not in ('encerrar', 'reiniciar'):
                    raise ValueError('Informe a decisão da colisão grave.')
                expected = 'reiniciar' if index < len(attempts) - 1 else 'encerrar'
                if incident['decision'] != expected:
                    raise ValueError('Colisão grave: efetue o reinício ou registre encerramento.')
            if incident['type'] in ('colisao_grave', 'desclassificacao') and attempt['completed']:
                raise ValueError('Uma tentativa encerrada por colisão grave/desclassificação não pode estar concluída.')
            if incident['type'] == 'desclassificacao' and index < len(attempts) - 1:
                raise ValueError('Uma desclassificação não permite reinício.')
        if len(severe) > 1:
            raise ValueError('Uma colisão grave encerra a tentativa; registre a próxima na tentativa reiniciada.')
        adjustments = attempt['adjustments']
        if len({a['id'] for a in adjustments}) != len(adjustments):
            raise ValueError('Ajustes duplicados.')
        for incident in attempt['incidents']:
            if incident['type'] == 'fita' and not any(a['id'] == incident['id'] for a in adjustments):
                raise ValueError('Colisão com fita exige ajuste de arbitragem.')
        for adj in adjustments:
            number(adj['points'], 'Ajuste em pontos', -1e12)
            if type(adj['resolved']) is not bool or (adj['resolved'] and not adj['reason'].strip()):
                raise ValueError('Ajustes resolvidos exigem justificativa.')


def score(record):
    validate(record)
    rules, attempts = record['rules'], record['attempts']
    active = attempts[-1]
    elapsed = sum(a['elapsed_s'] for a in attempts)
    base = len(active['arrivals']) * rules['arrival'] + len(active['objects']) * sum(rules[k] for k in ('identified', 'captured', 'delivered'))
    losses = []
    for area, ok in active['arrivals'].items():
        if not ok:
            losses.append({'kind': 'tarefa', 'cause': 'Chegada não realizada', 'area': area, 'object': None, 'points': rules['arrival']})
    for obj in active['objects']:
        for key, label in [('identified', 'Identificação'), ('captured', 'Captura'), ('delivered', 'Entrega')]:
            if not obj[key]:
                losses.append({'kind': 'tarefa', 'cause': f'{label} não realizada',
                               'area': obj['destination'] if key == 'delivered' else obj['source'],
                               'object': obj['id'], 'points': rules[key]})
    missing = sum(l['points'] for l in losses)
    completed = active['completed']
    perfect = completed and elapsed <= record['limit_s'] and not missing
    perfect_bonus = len(active['objects']) * rules['perfect_per_object'] if perfect else 0
    time_bonus = max(0, record['limit_s'] - elapsed) * rules['time_per_second'] if completed and elapsed <= record['limit_s'] else 0
    seen_processes = set()
    penalty = 0
    for incident in active['incidents']:
        kind = incident['type']
        points = rules['penalties'].get(kind, 0)
        if kind == 'manipulacao_deduzida':
            if incident['process'] in seen_processes:
                continue
            seen_processes.add(incident['process'])
        if points:
            penalty += points
            losses.append({'kind': 'penalidade', 'cause': incident['cause'] or INCIDENTS[kind],
                           'area': incident['area'], 'object': incident['object'], 'points': points})
    # Uma decisão anterior ainda pendente precisa ser revisada, mesmo com pontos descartados.
    pending = [a['label'] for attempt in attempts for a in attempt['adjustments'] if not a['resolved']]
    adjustment = sum(a['points'] for a in active['adjustments'] if a['resolved'])
    for adj in active['adjustments']:
        if adj['resolved'] and adj['points'] < 0:
            losses.append({'kind': 'ajuste', 'cause': adj['label'], 'area': '', 'object': None, 'points': -adj['points']})
    subtotal = base - missing - penalty + perfect_bonus + time_bonus + adjustment
    severe = any(i['type'] == 'colisao_grave' and i['decision'] == 'encerrar' for i in active['incidents'])
    severe_loss = subtotal * 0.5 if severe else 0
    total = subtotal - severe_loss
    disqualified = any(i['type'] == 'desclassificacao' for a in attempts for i in a['incidents'])
    if severe_loss:
        losses.append({'kind': 'penalidade', 'cause': 'Colisão grave (50%)', 'area': '', 'object': None, 'points': severe_loss})
    if disqualified:
        total = 0
    return {'base': base, 'missing': missing, 'penalties': penalty,
            'perfect_bonus': perfect_bonus, 'time_bonus': time_bonus,
            'adjustments': adjustment, 'severe_loss': severe_loss, 'total': total,
            'elapsed_s': elapsed, 'completed': completed, 'perfect': perfect,
            'pending': pending, 'disqualified': disqualified, 'losses': losses}


def comparison_key(record):
    return (record['scenario']['mission'], record['seed'], record['scenario']['hash'],
            record['limit_s'], canonical(record['rules']))
