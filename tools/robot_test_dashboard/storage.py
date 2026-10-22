"""SQLite transacional com revisões imutáveis e backups portáveis."""
from __future__ import annotations

from contextlib import closing
import csv
from datetime import datetime
import io
import json
import os
from pathlib import Path
import sqlite3

from domain import TZ, canonical, now, score, validate


class Store:
    def __init__(self, path=None):
        default = Path(__file__).parent / 'data' / 'tests.sqlite3'
        self.path = Path(path or os.environ.get('ROBOT_TEST_DB', default)).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with closing(self.connect()) as db, db:
            db.execute('''CREATE TABLE IF NOT EXISTS revisions (
                record_id TEXT NOT NULL, revision INTEGER NOT NULL,
                saved_at TEXT NOT NULL, status TEXT NOT NULL,
                reason TEXT NOT NULL, payload TEXT NOT NULL, score TEXT NOT NULL,
                PRIMARY KEY(record_id, revision))''')
            db.execute('CREATE TABLE IF NOT EXISTS preferences (key TEXT PRIMARY KEY, value TEXT NOT NULL)')

    def generation_settings(self):
        with closing(self.connect()) as db:
            row = db.execute("SELECT value FROM preferences WHERE key='generation'").fetchone()
        return json.loads(row['value']) if row else {}

    def save_generation_setting(self, key, value):
        with closing(self.connect()) as db, db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute("SELECT value FROM preferences WHERE key='generation'").fetchone()
            settings = json.loads(row['value']) if row else {}
            settings[key] = value
            db.execute('INSERT OR REPLACE INTO preferences VALUES (?, ?)', ('generation', json.dumps(settings)))

    def connect(self):
        db = sqlite3.connect(self.path, timeout=15)
        db.row_factory = sqlite3.Row
        return db

    @staticmethod
    def decode(row):
        if row is None:
            return None
        result = dict(row)
        result['payload'] = json.loads(result['payload'])
        result['score'] = json.loads(result['score'])
        return result

    def latest(self, record_id):
        with closing(self.connect()) as db:
            return self.decode(db.execute('SELECT * FROM revisions WHERE record_id=? ORDER BY revision DESC LIMIT 1', (record_id,)).fetchone())

    def history(self, record_id):
        with closing(self.connect()) as db:
            return [self.decode(r) for r in db.execute('SELECT * FROM revisions WHERE record_id=? ORDER BY revision DESC', (record_id,))]

    def list_latest(self):
        with closing(self.connect()) as db:
            return [self.decode(r) for r in db.execute('''SELECT r.* FROM revisions r
                JOIN (SELECT record_id, MAX(revision) AS rev FROM revisions GROUP BY record_id) last
                ON r.record_id=last.record_id AND r.revision=last.rev ORDER BY r.saved_at DESC''')]

    def save(self, payload, *, status='draft', expected_revision=0, reason='', reviewed=False):
        if status not in ('draft', 'finalized', 'void'):
            raise ValueError('Situação inválida.')
        if status == 'finalized' and not reviewed:
            raise ValueError('Revise a ficha antes de finalizar.')
        if status == 'void' and not reason.strip():
            raise ValueError('Informe a justificativa da anulação.')
        calculated = score(payload)
        with closing(self.connect()) as db, db:
            db.execute('BEGIN IMMEDIATE')
            latest = db.execute('SELECT revision, status FROM revisions WHERE record_id=? ORDER BY revision DESC LIMIT 1', (payload['id'],)).fetchone()
            actual = latest['revision'] if latest else 0
            if actual != expected_revision:
                raise ValueError('O teste foi alterado em outra sessão. Reabra o registro no Histórico.')
            if latest and latest['status'] in ('finalized', 'void') and not reason.strip():
                raise ValueError('Uma correção exige justificativa.')
            revision = actual + 1
            db.execute('INSERT INTO revisions VALUES (?,?,?,?,?,?,?)',
                       (payload['id'], revision, now(), status, reason.strip(), canonical(payload), canonical(calculated)))
        return self.latest(payload['id'])

    def backup(self, *, force=False):
        directory = self.path.parent / 'backups'
        directory.mkdir(exist_ok=True)
        stamp = datetime.now(TZ).strftime('%Y-%m-%d_%H-%M-%S-%f' if force else '%Y-%m-%d')
        target = directory / f'tests-{stamp}.sqlite3'
        if target.exists():
            return target
        # Backup API preserva consistência mesmo se outra aba estiver salvando.
        with closing(self.connect()) as source, closing(sqlite3.connect(target)) as dest:
            source.backup(dest)
        return target

    def export_json(self):
        with closing(self.connect()) as db:
            rows = [self.decode(r) for r in db.execute('SELECT * FROM revisions ORDER BY record_id, revision')]
        return json.dumps({'format': 'robot-tests-v1', 'exported_at': now(), 'revisions': rows}, ensure_ascii=False, indent=2, allow_nan=False)

    def restore(self, text):
        try:
            exported = json.loads(text)
            if not isinstance(exported, dict) or exported.get('format') != 'robot-tests-v1' or not isinstance(exported.get('revisions'), list):
                raise ValueError('Formato de backup incompatível.')
            rows = exported['revisions']
            keys = set()
            for row in rows:
                key = (row['record_id'], row['revision'])
                if key in keys or type(row['revision']) is not int or row['revision'] < 1 or row['status'] not in ('draft', 'finalized', 'void'):
                    raise ValueError('Revisão inválida ou duplicada no backup.')
                keys.add(key)
                validate(row['payload'])
                if row['record_id'] != row['payload']['id'] or canonical(score(row['payload'])) != canonical(row['score']):
                    raise ValueError('Pontuação ou identificação inconsistente no backup.')
                datetime.fromisoformat(row['saved_at'])
                if row['status'] == 'void' and not row['reason'].strip():
                    raise ValueError('Anulação sem justificativa.')
        except (KeyError, TypeError, AttributeError, json.JSONDecodeError, OverflowError) as exc:
            raise ValueError('Backup inválido ou incompleto.') from exc
        self.backup(force=True)
        added = 0
        with closing(self.connect()) as db, db:
            db.execute('BEGIN IMMEDIATE')
            for row in rows:
                previous = db.execute('SELECT * FROM revisions WHERE record_id=? AND revision=?', (row['record_id'], row['revision'])).fetchone()
                if previous:
                    if self.decode(previous) != row:
                        raise ValueError('Conflito: o banco já contém uma revisão diferente com o mesmo ID. Nada foi restaurado.')
                    continue
                db.execute('INSERT INTO revisions VALUES (?,?,?,?,?,?,?)',
                           (row['record_id'], row['revision'], row['saved_at'], row['status'], row['reason'], canonical(row['payload']), canonical(row['score'])))
                added += 1
        return added

    def csv_exports(self):
        """Três tabelas: revisões, resultados por tarefa e incidentes/ajustes."""
        exported = json.loads(self.export_json())['revisions']
        summaries, tasks, incidents = [], [], []
        for row in exported:
            p, s = row['payload'], row['score']
            common = {'teste': row['record_id'], 'revisao': row['revision'],
                      'missao': p['scenario']['mission'], 'seed': p['seed']}
            summaries.append({**common, 'data': p['tested_at'], 'situacao': row['status'],
                              'cenario_hash': p['scenario']['hash'], 'limite_s': p['limit_s'],
                              'tempo_s': s['elapsed_s'], 'pontos': s['total'],
                              'concluido': s['completed'], 'perfeito': s['perfect'],
                              'pendente': bool(s['pending']), 'justificativa': row['reason'], 'observacoes': p['notes']})
            for n, attempt in enumerate(p['attempts'], 1):
                for area, ok in attempt['arrivals'].items():
                    tasks.append({**common, 'tentativa': n, 'objeto': '', 'area': area, 'tarefa': 'chegada', 'realizada': ok})
                for obj in attempt['objects']:
                    for kind in ('identified', 'captured', 'delivered'):
                        tasks.append({**common, 'tentativa': n, 'objeto': obj['id'],
                                      'area': obj['destination'] if kind == 'delivered' else obj['source'],
                                      'tarefa': kind, 'realizada': obj[kind]})
                for item in attempt['incidents']:
                    incidents.append({**common, 'tentativa': n, 'categoria': 'incidente', **item})
                for item in attempt['adjustments']:
                    incidents.append({**common, 'tentativa': n, 'categoria': 'ajuste', **item})

        def encode(rows):
            out = io.StringIO(newline='')
            fields = list(dict.fromkeys(k for row in rows for k in row)) or ['teste']
            writer = csv.DictWriter(out, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                # CSV pode ser aberto em planilha: escape fórmulas em texto livre.
                writer.writerow({k: "'" + v if isinstance(v, str) and v.startswith(('=', '+', '-', '@', '\t', '\r')) else v for k, v in row.items()})
            return '\ufeff' + out.getvalue()

        return {'revisoes.csv': encode(summaries), 'tarefas.csv': encode(tasks), 'incidentes.csv': encode(incidents)}
