"""Formulário persistente e acompanhamento das gerações em segundo plano."""
from pathlib import Path

import streamlit as st

from planner_service import ensure_service

from domain import MISSIONS, import_scenario, new_record
from generation import (DEFAULTS, INSTALLED_PLANS_DIRECTORY, MISSION_PLAN_IDS, PLANS_DIRECTORY,
                        read_status, save_plan, start_generation)


def generation_form(store, activate):
    st.subheader('Gerar cenário e plano')
    st.caption('Os campos são salvos automaticamente neste computador, inclusive ao reiniciar o painel.')
    saved = {**DEFAULTS, **store.generation_settings()}

    def changed(name):
        store.save_generation_setting(name, st.session_state['generation:' + name])

    def text(name, label, help=None):
        return st.text_input(label, value=saved[name], key='generation:' + name,
                             help=help, on_change=changed, args=(name,))

    def select(name, label, options):
        value = saved[name] if saved[name] in options else options[0]
        return st.selectbox(label, options, index=options.index(value), key='generation:' + name,
                            on_change=changed, args=(name,))

    settings = {}
    a, b, c = st.columns(3)
    with a:
        settings['mission'] = select('mission', 'Prova', sorted(MISSIONS))
    with b:
        settings['seed'] = text('seed', 'Seed desejada')
    with c:
        settings['decoys'] = text('decoys', 'Quantidade de decoys (opcional)', 'Vazio usa o padrão do Commander.')
    with st.expander('Caminhos e opções de planejamento'):
        settings['commander'] = text('commander', 'Pasta do Commander')
        settings['python'] = text('python', 'Python do Commander')
        settings['metaarena'] = text('metaarena', 'Metaarena JSON (opcional)', 'Vazio usa a metaarena configurada no Commander.')
        settings['setup'] = text('setup', 'Setup ROS do workspace')
        settings['arena'] = text('arena', 'Arena YAML')
        settings['map'] = text('map', 'Mapa YAML', 'A máscara <nome_do_mapa>_keepout.yaml deve estar na mesma pasta.')
        settings['planner_params'] = text('planner_params', 'Parâmetros do planner YAML')
        settings['plans_directory'] = text('plans_directory', 'Pasta para salvar os planos',
                                          'Na pasta padrão, também atualiza os planos instalados do mission_manager.')
        settings['objective'] = select('objective', 'Objetivo da otimização', ['visits_then_distance', 'visits', 'distance'])
        settings['cargo_slots'] = text('cargo_slots', 'Compartimentos de carga', 'Nomes separados por espaços, por exemplo: left right.')
        settings['time_limit'] = text('time_limit', 'Limite da busca (segundos)', 'Após o cálculo das distâncias. O processo pode demorar mais que esse limite.')
        settings['max_states'] = text('max_states', 'Máximo de estados da busca', 'Se o otimizador atingir um limite, aumente este valor ou o tempo e gere novamente.')
    root = store.path.resolve().parent / 'generations'
    jobs = sorted((p for p in root.glob('*/status.json') if p.parent.name != 'planner'), reverse=True) if root.exists() else []
    running = any(read_status(path.parent)['state'] == 'running' for path in jobs)
    st.session_state['generation:busy'] = running
    planner_controls(root, settings)
    jobs = sorted((p for p in root.glob('*/status.json') if p.parent.name != 'planner'), reverse=True) if root.exists() else []
    if jobs:
        st.selectbox('Gerações salvas', [str(p.parent) for p in jobs],
                     format_func=lambda p: Path(p).name, key='generation:selected')
        generation_result(store, activate)


@st.fragment(run_every=2)
def generation_result(store, activate):
    directory = Path(st.session_state['generation:selected'])
    try:
        status = read_status(directory)
        running = any(read_status(path.parent)['state'] == 'running'
                      for path in directory.parent.glob('*/status.json') if path.parent.name != 'planner')
        if running != st.session_state.get('generation:busy', False):
            st.rerun()
        if status['state'] == 'running':
            st.info(status['message'])
        elif status['state'] == 'error':
            st.error(status['message'])
        else:
            st.success(status['message'])
            team = directory / status['team']
            scenario = import_scenario(team.read_text(encoding='utf-8'), team.name)
            st.caption(f"{scenario['mission']} · seed {status['seed']} · {len(scenario['objects'])} objetos · {len(scenario['areas'])} áreas")
            st.dataframe([{'Objeto': obj['id'], 'Origem': obj['source'], 'Destino': obj['destination']}
                          for obj in scenario['objects']], hide_index=True)
            if st.button('Criar rascunho com cenário e plano', key='generation:create:' + directory.name):
                plan = directory / status['plan']
                payload = new_record(scenario, status['seed'], {'name': plan.name, 'text': plan.read_text(encoding='utf-8')})
                activate(store.save(payload))
                st.rerun()
        st.caption('Arquivos: ' + str(directory))
        for path in sorted(directory.glob('*.yaml')):
            if path.name == status.get('plan'):
                destination = store.generation_settings().get('plans_directory', DEFAULTS['plans_directory'])
                st.caption('Destino do plano: ' + destination)
                st.caption('Substitui o arquivo da missão: ' + MISSION_PLAN_IDS[scenario['mission']] + '.yaml')
                saved_key = 'generation:saved_plan:' + directory.name
                if st.button('Salvar plano na pasta de missões', key='generation:save_plan:' + directory.name):
                    if not destination.strip():
                        raise ValueError('Informe a pasta para salvar os planos.')
                    installed = (INSTALLED_PLANS_DIRECTORY
                                 if Path(destination).expanduser().resolve() == PLANS_DIRECTORY.resolve()
                                 and INSTALLED_PLANS_DIRECTORY.parent.is_dir() else None)
                    plan_id, paths = save_plan(directory, destination, installed_directory=installed)
                    st.session_state[saved_key] = (plan_id, [str(p) for p in paths])
                if saved_key in st.session_state:
                    plan_id, paths = st.session_state[saved_key]
                    st.success('Plano salvo. ID para execução: ' + plan_id)
                    for saved_path in paths:
                        st.code(saved_path, language='text')
                continue
            st.download_button('Baixar ' + path.name, path.read_bytes(), file_name=path.name,
                               mime='application/x-yaml', key='generation:download:' + directory.name + path.name)
        with st.expander('Logs da geração'):
            for path in sorted(directory.glob('*.log')):
                st.caption(path.name)
                st.code(path.read_text(encoding='utf-8', errors='replace')[-20000:] or 'Aguardando saída…', language='text')
    except (OSError, ValueError) as exc:
        st.error(str(exc))


@st.fragment(run_every=2)
def planner_controls(root, settings):
    running = any(read_status(path.parent)['state'] == 'running'
                  for path in root.glob('*/status.json') if path.parent.name != 'planner')
    try:
        planner = ensure_service(root, settings, busy=running)
        if planner['state'] == 'ready':
            st.success(planner['message'])
        elif planner['state'] == 'error':
            st.error(planner['message'])
            if st.button('Tentar iniciar servidor novamente'):
                ensure_service(root, settings, busy=running, retry=True)
                st.rerun()
        else:
            st.info(planner['message'])
        if st.button('Gerar cenário e plano', type='primary',
                     disabled=running or planner['state'] != 'ready'):
            directory = start_generation(root, settings, planner=planner)
            st.session_state['generation:selected'] = str(directory)
            st.rerun()
        if running:
            st.info('Há uma geração em andamento. O acompanhamento é atualizado automaticamente.')
        with st.expander('Logs do servidor de planejamento'):
            directory = root / 'planner'
            paths = [directory / 'worker.log']
            from planner_service import service_status
            status = service_status(directory)
            if status.get('log'):
                paths.append(Path(status['log']))
            for path in paths:
                if path.is_file():
                    st.caption(path.name)
                    st.code(path.read_text(errors='replace')[-20000:] or 'Aguardando saída…', language='text')
    except (OSError, ValueError, KeyError) as exc:
        st.error('Não foi possível preparar o servidor: ' + str(exc))
