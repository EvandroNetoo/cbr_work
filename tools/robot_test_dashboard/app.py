"""Painel local de testes. Inicie com bash run.sh."""
from copy import deepcopy
from datetime import datetime
from uuid import uuid4

import pandas as pd
import streamlit as st

from analytics import groups, summarize
from domain import (INCIDENTS, TZ, add_incident, canonical, import_scenario,
                    new_record, restart, score)
from storage import Store


st.set_page_config(page_title='Bancada de testes · CBR', page_icon='🤖', layout='wide')
st.markdown('''<style>
    .stApp {background: #f5f8fc; color: #15324f;}
    h1,h2,h3 {font-family: "DejaVu Sans", sans-serif; color: #15324f;}
    .block-container {padding-top: 2rem; max-width: 1300px;}
    [data-testid="stSidebar"] {background: #e7eff8;}
    [data-testid="stMetric"] {border-left: 4px solid #2463a6; padding-left: 1rem;}
    button:focus-visible, input:focus-visible {outline: 3px solid #ba630c;}
</style>''', unsafe_allow_html=True)

STATUS = {'draft': 'Rascunho', 'finalized': 'Finalizado', 'void': 'Anulado'}


def label(row):
    p = row['payload']
    return f"{p['scenario']['mission']} / seed {p['seed']} — {datetime.fromisoformat(p['tested_at']).strftime('%d/%m/%Y %H:%M')} — {STATUS[row['status']]} — {row['record_id'][:8]}"


def activate(row):
    st.session_state['work'] = deepcopy(row['payload'])
    st.session_state['work_revision'] = row['revision']
    st.session_state['work_status'] = row['status']
    st.session_state['editor_epoch'] = st.session_state.get('editor_epoch', 0) + 1


def persist(store, payload, status='draft', reason='', reviewed=False):
    row = store.save(payload, status=status,
                     expected_revision=st.session_state['work_revision'],
                     reason=reason, reviewed=reviewed)
    st.session_state['work_revision'] = row['revision']
    st.session_state['work_status'] = row['status']
    if status == 'draft':
        key = f"{payload['id']}:{st.session_state['editor_epoch']}:{len(payload['attempts'])}:reviewed"
        st.session_state[key] = False
    return row


def score_view(calculated):
    a, b, c, d = st.columns(4)
    a.metric('Pontuação provisória' if calculated['pending'] else 'Pontuação calculada', f"{calculated['total']:g}")
    b.metric('Pontos não conquistados', f"−{calculated['missing']:g}")
    c.metric('Penalidades fixas', f"−{calculated['penalties']:g}")
    d.metric('Tempo acumulado', f"{calculated['elapsed_s'] / 60:.2f} min")
    st.caption(
        f"Base {calculated['base']:g} − tarefas {calculated['missing']:g} "
        f"− penalidades {calculated['penalties']:g} + perfeição {calculated['perfect_bonus']:g} "
        f"+ tempo {calculated['time_bonus']:g} + ajustes ({calculated['adjustments']:g}) "
        f"− colisão grave {calculated['severe_loss']:g} = {calculated['total']:g}"
    )
    if calculated['disqualified']:
        st.warning('Desclassificação: pontuação final zerada.')
    if calculated['pending']:
        st.warning('Resultado provisório. Resolva as decisões de arbitragem: ' + ', '.join(calculated['pending']))
    if calculated['losses']:
        with st.expander('Ver deduções por tarefa e incidente'):
            st.dataframe(pd.DataFrame(calculated['losses']).rename(columns={
                'kind': 'Tipo', 'cause': 'Causa', 'area': 'Área', 'object': 'Objeto', 'points': 'Pontos perdidos'}), hide_index=True)


def import_form(store):
    st.header('Novo teste')
    st.write('Importe o cenário da equipe. A ficha começa com as tarefas realizadas; marque as falhas durante a revisão.')
    upload = st.file_uploader('Cenário do Commander (*_equipe.yaml)', type=['yaml', 'yml'], key='scenario_upload')
    if upload is None:
        return
    try:
        scenario = import_scenario(upload.getvalue().decode('utf-8-sig'), upload.name)
    except (ValueError, UnicodeError) as exc:
        st.error(str(exc))
        return
    st.caption(f"{scenario['mission']} · {len(scenario['objects'])} objetos · {len(scenario['areas'])} áreas · {scenario['duration_s']/60:g} minutos")
    seed = st.text_input('Confirme a seed testada', value=scenario['seed'], key=f"seed:{scenario['hash']}")
    plan = st.file_uploader('Plano otimizado usado (opcional)', type=['yaml', 'yml'], key='plan_upload')
    current = st.session_state.get('work')
    if current:
        st.info('A ficha atual permanece salva. Criar outro teste fecha a edição atual.')
    if st.button('Criar rascunho', type='primary'):
        try:
            attachment = {'name': plan.name, 'text': plan.getvalue().decode('utf-8-sig')} if plan else None
            payload = new_record(scenario, seed, attachment)
            activate(store.save(payload))
            st.rerun()
        except (ValueError, UnicodeError) as exc:
            st.error(str(exc))


def incident_editor(store, payload, attempt, prefix):
    st.subheader('Incidentes e arbitragem')
    areas = payload['scenario']['areas']
    objects = [o['id'] for o in attempt['objects']]
    with st.expander('Adicionar incidente'):
        with st.form(prefix + ':incident', clear_on_submit=True):
            kind = st.selectbox('Tipo de incidente', list(INCIDENTS), format_func=INCIDENTS.get)
            a, b = st.columns(2)
            area = a.selectbox('Área do incidente', [''] + areas, format_func=lambda x: x or 'Não informada')
            obj = b.selectbox('Objeto do incidente', [None] + objects, format_func=lambda x: 'Não informado' if x is None else str(x))
            cause = st.text_input('Causa / observação')
            process = st.text_input('Identificador do processo (obrigatório para manipulação deduzida)',
                                    help='Ex.: captura_objeto4_WS2_tentativa1. Repetições no mesmo processo cobram uma única dedução; uma nova captura ou depósito usa outro identificador.')
            decision = st.selectbox('Decisão para colisão grave', ['encerrar', 'reiniciar'],
                                    format_func=lambda x: 'Encerrar com redução de 50%' if x == 'encerrar' else 'Reiniciar a rodada')
            if st.form_submit_button('Adicionar incidente'):
                try:
                    if kind == 'colisao_grave' and any(i['type'] == kind for i in attempt['incidents']):
                        raise ValueError('Esta tentativa já tem uma colisão grave.')
                    if kind == 'colisao_grave' and len(payload['attempts']) == 2 and decision == 'reiniciar':
                        raise ValueError('O reinício já foi utilizado. A segunda colisão grave encerra a rodada.')
                    if kind == 'manipulacao_deduzida' and not process.strip():
                        raise ValueError('Informe o identificador do processo.')
                    if kind == 'colisao_grave' and decision == 'reiniciar' and any(i['type'] == 'desclassificacao' for i in attempt['incidents']):
                        raise ValueError('Uma desclassificação não permite reinício.')
                    add_incident(attempt, kind, area=area, obj=obj, cause=cause, process=process, decision=decision)
                    if kind in ('colisao_grave', 'desclassificacao'):
                        attempt['completed'] = False
                        st.session_state[prefix + ':completed'] = False
                    if kind == 'colisao_grave' and decision == 'reiniciar':
                        restart(payload)
                        st.session_state['editor_epoch'] += 1
                    # Persistir antes do rerun evita perder incidentes no recarregamento.
                    persist(store, payload)
                    st.rerun()
                except ValueError as exc:
                    st.error(str(exc))
    for item in list(attempt['incidents']):
        cols = st.columns([8, 1])
        cols[0].write(f"{INCIDENTS[item['type']]} | área: {item['area'] or '—'} | objeto: {item['object'] or '—'} | {item['cause']}")
        if item['process']:
            cols[0].caption('Processo: ' + item['process'])
        if cols[1].button('Remover', key=prefix + ':remove:' + item['id']):
            attempt['incidents'].remove(item)
            attempt['adjustments'] = [a for a in attempt['adjustments'] if a['id'] != item['id']]
            persist(store, payload)
            st.rerun()
    st.caption('Percentuais e bônus sem marcações: registre o valor convertido em pontos e explique a base utilizada. Valores negativos deduzem; positivos acrescentam.')
    with st.form(prefix + ':adjustment', clear_on_submit=True):
        category = st.selectbox('Novo ajuste de arbitragem', ['Bônus de navegação sem marcações', 'Simplificação', 'Outra decisão'])
        if st.form_submit_button('Registrar decisão pendente'):
            attempt['adjustments'].append({'id': str(uuid4()), 'label': category, 'resolved': False, 'points': 0, 'reason': ''})
            persist(store, payload)
            st.rerun()
    adjustment_editor(store, payload, attempt, prefix)


def adjustment_editor(store, payload, attempt, prefix):
    for adjustment in list(attempt['adjustments']):
        key = prefix + ':adj:' + adjustment['id']
        with st.expander(adjustment['label'], expanded=not adjustment['resolved']):
            adjustment['points'] = st.number_input('Valor em pontos (+ / −)', value=float(adjustment['points']), key=key + ':points')
            adjustment['reason'] = st.text_area('Justificativa e base do cálculo', value=adjustment['reason'], key=key + ':reason')
            checked = st.checkbox('Decisão resolvida', value=adjustment['resolved'], key=key + ':resolved')
            adjustment['resolved'] = checked and bool(adjustment['reason'].strip())
            if checked and not adjustment['reason'].strip():
                st.warning('Preencha a justificativa para resolver a decisão.')
            if not any(i['id'] == adjustment['id'] for i in attempt['incidents']):
                if st.button('Remover ajuste', key=key + ':remove'):
                    attempt['adjustments'].remove(adjustment)
                    persist(store, payload)
                    st.rerun()


def task_editor(store, payload, attempt, prefix):
    st.subheader('Tarefas da tentativa atual')
    st.caption('Os resultados são independentes. Uma queda não apaga uma captura que já aconteceu.')
    choices = {}
    for area in attempt['arrivals']:
        choices[f'Chegada · {area}'] = ('arrival', area)
    for obj in attempt['objects']:
        for name, label_text in [('identified', 'Identificar'), ('captured', 'Capturar'), ('delivered', 'Entregar')]:
            choices[f"{label_text} · objeto {obj['id']}"] = (name, obj['id'])
    remaining = st.multiselect('Tarefas que ficaram sem realizar ao interromper', list(choices), key=prefix + ':remaining')
    if st.button('Marcar selecionadas como não realizadas', key=prefix + ':mark_remaining', disabled=not remaining):
        for selected in remaining:
            kind, identifier = choices[selected]
            if kind == 'arrival':
                attempt['arrivals'][identifier] = False
                st.session_state[prefix + ':arrival:' + identifier] = False
            else:
                next(o for o in attempt['objects'] if o['id'] == identifier)[kind] = False
                st.session_state[f'{prefix}:{kind}:{identifier}'] = False
        attempt['completed'] = False
        st.session_state[prefix + ':completed'] = False
        persist(store, payload)
        st.rerun()
    cols = st.columns(4)
    for n, area in enumerate(attempt['arrivals']):
        attempt['arrivals'][area] = cols[n % 4].checkbox('Chegou a ' + area, value=attempt['arrivals'][area], key=prefix + ':arrival:' + area)
    st.markdown('**Objeto e transporte esperado**')
    for obj in attempt['objects']:
        a, b, c, d = st.columns([3, 2, 2, 2])
        a.write(f"**{obj['id']}** · {obj['source']} → {obj['destination']}")
        for col, key, text in [(b, 'identified', 'Identificado'), (c, 'captured', 'Capturado'), (d, 'delivered', 'Entrega correta')]:
            obj[key] = col.checkbox(text, value=obj[key], key=f"{prefix}:{key}:{obj['id']}")


def render_editor(store):
    payload = st.session_state.get('work')
    if not payload:
        return
    if st.session_state['work_status'] != 'draft':
        st.success('Registro salvo. Abra uma correção pelo Histórico se precisar alterar a ficha.')
        score_view(store.latest(payload['id'])['score'])
        return
    st.divider()
    st.header(f"Ficha · {payload['scenario']['mission']} / seed {payload['seed']}")
    st.caption('Rascunho com salvamento automático a cada alteração. Não participa das estatísticas até ser revisado e finalizado.')
    prefix = f"{payload['id']}:{st.session_state['editor_epoch']}:{len(payload['attempts'])}"
    date = datetime.fromisoformat(payload['tested_at']).astimezone(TZ)
    a, b, c = st.columns(3)
    day = a.date_input('Data do teste', value=date.date(), key=prefix + ':day')
    clock = b.time_input('Hora do teste (São Paulo)', value=date.time().replace(microsecond=0), key=prefix + ':clock')
    payload['tested_at'] = datetime.combine(day, clock, tzinfo=TZ).isoformat()
    payload['limit_s'] = c.number_input('Limite (minutos)', min_value=1/60, value=float(payload['limit_s']/60), key=prefix + ':limit') * 60
    if payload['scenario']['mission'] == 'FINAL' and payload['limit_s'] != 1200:
        st.info('Treino com limite diferente do PDF CBR 2026: a final no regulamento tem 20 minutos. Este teste usa o limite indicado acima e será comparado somente com testes do mesmo limite.')
    attempt = payload['attempts'][-1]
    attempt['elapsed_s'] = st.number_input(f"Tempo consumido nesta tentativa (segundos) · tentativa {len(payload['attempts'])}", min_value=0.0, value=float(attempt['elapsed_s']), key=prefix + ':elapsed')
    fatal = any(i['type'] in ('colisao_grave', 'desclassificacao') for i in attempt['incidents'])
    if len(payload['attempts']) == 2:
        previous = payload['attempts'][0]
        st.info(f"Reinício utilizado. {previous['elapsed_s']:g} s da primeira tentativa continuam consumidos; seus pontos foram descartados.")
        with st.expander('Consultar a primeira tentativa preservada'):
            previous['elapsed_s'] = st.number_input('Tempo consumido na primeira tentativa (segundos)', min_value=0.0,
                                                    value=float(previous['elapsed_s']), key=prefix + ':previous_elapsed')
            st.json(previous)
        if previous['adjustments']:
            st.caption('Decisões da primeira tentativa: permanecem no histórico, com seus pontos descartados após o reinício.')
            adjustment_editor(store, payload, previous, prefix + ':previous')
    task_editor(store, payload, attempt, prefix)
    incident_editor(store, payload, attempt, prefix)
    all_tasks = all(attempt['arrivals'].values()) and all(o[k] for o in attempt['objects'] for k in ('identified', 'captured', 'delivered'))
    if fatal or not all_tasks:
        attempt['completed'] = False
        st.session_state[prefix + ':completed'] = False
    attempt['completed'] = st.checkbox('Missão concluída (todas as tarefas)', value=attempt['completed'], disabled=fatal or not all_tasks, key=prefix + ':completed')
    payload['notes'] = st.text_area('Observações do teste', value=payload['notes'], key=prefix + ':notes')
    try:
        calculated = score(payload)
        saved = store.latest(payload['id'])
        if canonical(saved['payload']) != canonical(payload):
            persist(store, payload)
        st.caption(f"Salvo automaticamente · revisão {st.session_state['work_revision']}")
    except ValueError as exc:
        st.error(str(exc))
        return
    score_view(calculated)
    st.sidebar.metric('Prévia da ficha aberta', f"{calculated['total']:g} pontos")
    st.sidebar.caption('Rascunho · ainda fora das estatísticas')
    with st.expander('Regras e interpretações deste teste'):
        st.json(payload['rules'])
    reviewed = st.checkbox('Revisei as tarefas, o tempo, os incidentes e a memória de cálculo', key=prefix + ':reviewed')
    if st.button('Finalizar teste', type='primary', disabled=not reviewed, key=prefix + ':finalize'):
        try:
            persist(store, payload, status='finalized', reviewed=True)
            st.rerun()
        except ValueError as exc:
            st.error(str(exc))
    if len(payload['attempts']) == 1 and not fatal:
        st.caption('O reinício abaixo descarta os pontos desta tentativa e mantém seu tempo consumido.')
        confirm = st.checkbox('Confirmo o reinício da rodada', key=prefix + ':confirm_restart')
        if st.button('Reiniciar rodada', disabled=not confirm, key=prefix + ':restart'):
            try:
                restart(payload)
                persist(store, payload)
                st.session_state['editor_epoch'] += 1
                st.rerun()
            except ValueError as exc:
                st.error(str(exc))


def archive_controls(store):
    with st.expander('Exportar ou restaurar histórico'):
        st.download_button('Exportar histórico completo (JSON)', store.export_json(), 'historico_robo.json', 'application/json')
        for name, data in store.csv_exports().items():
            st.download_button('Exportar ' + name, data, name, 'text/csv', key='export:' + name)
        upload = st.file_uploader('Restaurar exportação JSON', type=['json'], key='restore_upload')
        st.caption('A restauração combina os históricos sem apagar dados. Conflitos cancelam toda a importação; um backup do banco é criado antes.')
        if st.button('Restaurar histórico', disabled=upload is None):
            try:
                added = store.restore(upload.getvalue().decode('utf-8-sig'))
                st.success(f'{added} revisões restauradas.')
            except (ValueError, UnicodeError) as exc:
                st.error(str(exc))


def history_page(store):
    st.header('Histórico')
    archive_controls(store)
    rows = store.list_latest()
    if not rows:
        st.info('Crie o primeiro teste na tela Novo teste.')
        return
    a, b, c = st.columns(3)
    mission = a.selectbox('Missão', ['Todas'] + sorted({r['payload']['scenario']['mission'] for r in rows}))
    seed = b.selectbox('Seed', ['Todas'] + sorted({r['payload']['seed'] for r in rows}))
    status = c.selectbox('Situação', ['Todas', 'Rascunho', 'Finalizado', 'Provisório', 'Anulado'])
    dates = [datetime.fromisoformat(r['payload']['tested_at']).date() for r in rows]
    period = st.date_input('Período', value=(min(dates), max(dates)))
    rows = [r for r in rows if (mission == 'Todas' or r['payload']['scenario']['mission'] == mission)
            and (seed == 'Todas' or r['payload']['seed'] == seed)
            and (status == 'Todas' or (status == 'Provisório' and r['score']['pending']) or STATUS[r['status']] == status)
            and (len(period) != 2 or period[0] <= datetime.fromisoformat(r['payload']['tested_at']).date() <= period[1])]
    if not rows:
        st.info('Nenhum teste nesse filtro.')
        return
    selected = st.selectbox('Escolha um teste', [r['record_id'] for r in rows], format_func=lambda x: label(next(r for r in rows if r['record_id'] == x)))
    latest = store.latest(selected)
    revisions = store.history(selected)
    rev = st.selectbox('Revisão para consultar', [r['revision'] for r in revisions],
                       format_func=lambda x: f"Revisão {x} · {STATUS[next(r for r in revisions if r['revision'] == x)['status']]}")
    row = next(r for r in revisions if r['revision'] == rev)
    st.caption(f"Salvo em {row['saved_at']} | {row['reason'] or 'Sem justificativa adicional'}")
    score_view(row['score'])
    with st.expander('Ficha completa e regras preservadas'):
        st.json(row['payload'])
    for key, title in [('scenario', 'Cenário original'), ('plan', 'Plano original')]:
        attachment = row['payload'][key]
        if key == 'scenario':
            attachment = attachment['file']
        if attachment:
            st.download_button(title, attachment['text'], attachment['name'], 'text/yaml', key=f"file:{selected}:{rev}:{key}")
    if latest['status'] == 'draft':
        if st.button('Abrir rascunho atual para edição'):
            activate(latest)
            st.rerun()
    else:
        reason = st.text_input('Justificativa para corrigir ou anular o registro', key='revision_reason:' + selected)
        a, b = st.columns(2)
        if a.button('Criar revisão para correção', disabled=not reason.strip()):
            try:
                activate(store.save(latest['payload'], expected_revision=latest['revision'], reason=reason))
                st.rerun()
            except ValueError as exc:
                st.error(str(exc))
        if b.button('Anular registro', disabled=not reason.strip() or latest['status'] == 'void'):
            try:
                store.save(latest['payload'], status='void', expected_revision=latest['revision'], reason=reason)
                if st.session_state.get('work', {}).get('id') == selected:
                    st.session_state.pop('work', None)
                st.rerun()
            except ValueError as exc:
                st.error(str(exc))


def statistics_page(store):
    st.header('Estatísticas')
    sets = groups(store.list_latest())
    if not sets:
        st.info('Finalize um teste revisado e resolva as pendências para começar a comparar os resultados.')
        return
    keys = list(sets)
    index = st.selectbox('Cenário comparável', range(len(keys)), format_func=lambda i: (
        f'{keys[i][0]} / seed {keys[i][1]} · cenário {keys[i][2][:10]} · {keys[i][3]/60:g} min · regras {sets[keys[i]][0]["payload"]["rules"]["version"]} · grupo {i+1}'))
    data = summarize(sets[keys[index]])
    st.caption(f"{data['count']} testes finalizados, sem pendências. Cenário, seed, limite e regras iguais.")
    a, b, c, d = st.columns(4)
    for col, title, value in [(a, 'Média', data['mean']), (b, 'Mediana', data['median']),
                               (c, 'Melhor resultado', data['best']), (d, 'Desvio padrão', data['stddev'])]:
        col.metric(title, f'{value:.1f}')
    a, b, c = st.columns(3)
    a.metric('Taxa de conclusão', f"{data['completion_rate']:.0%}")
    b.metric('Rodadas perfeitas', f"{data['perfect_rate']:.0%}")
    c.metric('Tempo médio das conclusões', '—' if data['mean_completed_s'] is None else f"{data['mean_completed_s']/60:.2f} min")
    timeline = pd.DataFrame(data['timeline'])
    timeline['data'] = pd.to_datetime(timeline['data'], utc=True).dt.tz_convert('America/Sao_Paulo')
    st.subheader('Evolução da pontuação')
    st.line_chart(timeline, x='data', y='pontos')
    st.subheader('Tempos das missões concluídas')
    completed = [r for r in sets[keys[index]] if r['score']['completed']]
    if completed:
        st.dataframe(pd.DataFrame([{'Data': r['payload']['tested_at'], 'Tempo (s)': r['score']['elapsed_s']} for r in completed]), hide_index=True)
    else:
        st.caption('Nenhuma missão concluída neste grupo.')
    st.subheader('Principais perdas acumuladas')
    if data['losses']:
        frame = pd.DataFrame(data['losses']).rename(columns={'area': 'Área', 'object': 'Objeto', 'cause': 'Causa', 'points': 'Pontos perdidos'})
        st.dataframe(frame, hide_index=True)
        st.bar_chart(frame.groupby('Causa', as_index=False)['Pontos perdidos'].sum(), x='Causa', y='Pontos perdidos')
    else:
        st.success('Nenhuma perda registrada nesses testes.')


store = Store()
try:
    backup_path = store.backup()
except OSError as exc:
    st.error(f'Não foi possível criar o backup diário: {exc}')
    st.stop()
with st.sidebar:
    st.title('Bancada CBR')
    st.caption('Missões, seeds e resultados do robô')
    page = st.radio('Navegação', ['Novo teste', 'Histórico', 'Estatísticas'])
    st.caption('Dados locais · sem conexão com o robô')
    st.caption('Backup diário: ' + backup_path.name)
    if 'work' in st.session_state:
        st.caption('Ficha aberta: ' + st.session_state['work']['id'][:8])
        if st.button('Fechar ficha salva'):
            st.session_state.pop('work', None)
            st.rerun()
try:
    if page == 'Novo teste':
        if 'work' in st.session_state:
            with st.expander('Criar outro teste'):
                import_form(store)
        else:
            import_form(store)
    elif page == 'Histórico':
        history_page(store)
    else:
        statistics_page(store)
    if page != 'Estatísticas':
        render_editor(store)
except (OSError, ValueError) as exc:
    st.error(str(exc))
