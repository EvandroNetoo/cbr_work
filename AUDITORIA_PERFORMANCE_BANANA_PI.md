# Auditoria de produção ROS 2 — CBR na Banana Pi

Data: 2026-08-20
Escopo: todo o código-fonte em `src/work`, launch/configuração de produção, interfaces com hardware e os serviços/plataforma em `MariolaZero`. Artefatos gerados em `build/`, `install/` e `log/` foram inspecionados apenas para detectar resíduos; não foram tratados como fonte.

## Como ler os resultados

- **ESTÁTICA**: conclusão derivada de código, configuração, grafo construído pelos launch files e custos calculáveis. É evidência concreta, mas não é uma medição de CPU na Banana Pi.
- **MEDIR NA BANANA**: hipótese bem fundamentada que ainda precisa de profiling no hardware ARM, com os periféricos reais e controle térmico.
- **CORRIGIDO**: alteração segura já aplicada nesta revisão e validada por build/testes locais.
- Estimativas de ganho são intervalos de engenharia, não promessas. O percentual global depende de qual cenário domina: repouso, locomoção, manipulação ou visão.

### Atualização implementada após profiling na Banana Pi

O `htop` confirmou cerca de 84–87% por núcleo com AprilTag contínuo e 36–39%
por núcleo sem visão. A partir desse baseline foram implementados, com rollback
por argumentos de launch:

- câmera 30→15 FPS; AprilTag permanece em 10 Hz, duas threads e
  `quad_decimate=1.0` para preservar precisão;
- `controller_manager` 50→30 Hz;
- braço com callback latest-value e um único ciclo serial a 30 Hz; comandos
  idênticos usam heartbeat de 5 Hz;
- base com leitura a 30 Hz e escrita deduplicada após quantização, também com
  heartbeat de 5 Hz;
- LiDAR e MoveIt permanecem sempre ativos por decisão desta fase.

As metas de aceite na Banana são redução de pelo menos 25% da CPU agregada com
visão e 30% em repouso, sem regressão de movimentos, parada ou detecção.

## 1. Resumo executivo

O sistema de produção sobe nove processos persistentes e quatro spawners transitórios. A arquitetura de `ros2_control` é conceitualmente boa, porém há duas pontes ROS adicionais entre cada `SystemInterface` e os drivers Python. Isso desacopla C++ de bibliotecas Python, mas hoje multiplica mensagens e, no braço, transforma cada ciclo de controle em uma escrita serial bloqueante.

O maior consumidor provável durante uma ação de visão era o AprilTag: a câmera entregava 30 FPS, `image_proc` copiava/retificava RGB e o detector executava pose estimation em todos os frames, com duas threads e `quad_decimate=1`. A câmera agora entrega 15 FPS e o caminho caro foi limitado a 10 detecções/s. Em repouso a câmera já é desligada pelo detector; portanto o líder provável passa a ser a dupla `controller_manager` + I/O do braço/base, seguida pelo `move_group` sempre residente e pelo LiDAR continuamente ativo sem consumidor versionado.

Não existe pacote de navegação/autonomia no fonte atual. `/scan_front` não tem assinante no projeto versionado. Há resíduos instalados de `navigation` e `motor_control`, mas não fonte correspondente. Logo, manter LiDAR, `move_group` e os quatro fluxos AprilTag sempre disponíveis deve ser decisão explícita, não efeito do launch monolítico.

Os cinco problemas de maior retorno são:

1. visão a 30 Hz com cópias RGB e detecção em todo frame — câmera reduzida a 15 Hz e detecção limitada a 10 Hz;
2. comando do braço bloqueando a callback até 50 vezes/s — substituído por latest-value e ciclo serial único a 30 Hz;
3. LiDAR continuamente ligado sem consumidor local — polling vazio já caiu de 100 para 20 Hz, mas o maior ganho é desligá-lo por demanda;
4. `move_group` e uma sequência de pick-and-place infinita — tornar planejamento/tarefa acionáveis sob demanda e executar uma missão finita;
5. taxas desalinhadas e repetição de comandos idênticos — controle alinhado em 30 Hz e heartbeat estacionário de 5 Hz.

O projeto ainda **não está pronto para uma imagem de produção imutável** porque a suíte completa termina com três falhas pré-existentes e o ambiente Python do sistema não encontra `serial`. Nenhuma das três falhas foi mascarada: duas podem ser testes desatualizados, mas a ausência de limites de segurança no URDF precisa de decisão técnica.

## 2. Top 10 suspeitos de CPU

Ranking predominantemente **ESTÁTICO**; deve ser confirmado pelo roteiro da seção 13.

| Rank | Processo/caminho | Situação | Evidência | Impacto esperado | Prioridade |
|---:|---|---|---|---|---|
| 1 | `apriltag_detector`: conversão, detecção e pose | Somente durante ação | Entrada 15 FPS; detector limitado a 10 Hz, 2 threads e sem redução espacial | Ainda alto durante visão, mas sem processar todos os frames | CRÍTICA — CORRIGIDO parcialmente |
| 2 | driver SO-101 + serial Feetech | Contínuo | ciclo único a 30 Hz faz `sync_write` por mudança/heartbeat e `sync_read` | Deve reduzir contenção e trabalho em repouso; medir na Banana | CRÍTICA — CORRIGIDO |
| 3 | `usb_cam` + `image_proc` + RGB→mono | Durante ação AprilTag | 320×240×RGB×15 = 3,46 MB/s por stream; raw + rect ≈6,91 MB/s, além de decode MJPEG | Alto em CPU/memória durante visão | ALTA — REDUZIDO |
| 4 | I/O da base Mariola | Contínuo | leitura 30 Hz; escrita apenas quando muda ou heartbeat 5 Hz | Futures e protocolo de leitura continuam relevantes | ALTA — REDUZIDO |
| 5 | `controller_manager` e controladores | Contínuo | update alinhado em 30 Hz; publica comando, estados, odometria e TF | Custo contínuo ainda mensurável | ALTA — REDUZIDO |
| 6 | LiDAR XV-11 | Contínuo | thread lê 115200 baud e processa byte a byte; `/scan_front` não tem assinante versionado (`lidar_driver.py:267-275`) | Médio; desperdício integral se não houver consumidor externo | ALTA |
| 7 | `move_group` | Contínuo após ativação | iniciado incondicionalmente após os controladores (`robot.launch.py:79,96-115`), mesmo sem nó autônomo no launch | CPU de fundo e memória; picos altos ao planejar | ALTA |
| 8 | pick-and-place repetitivo | Se iniciado manualmente | `while True` repete visão e múltiplos planejamentos (`pegar_e_colocar.py:44-180`); script nem é instalado por `setup.py` | Picos contínuos e missão sem término | ALTA, possivelmente não utilizado |
| 9 | menu da Mariola | Se serviço legado estiver habilitado | polling a 10 Hz e dois loops sem espera enquanto botão estava pressionado (`MariolaZero/menuPrincipal/start.py:31-65`) | Um core podia ir a 100% durante long press; polling de 10 ms aplicado | MÉDIA/ALTA — CORRIGIDO |
| 10 | filtro global de `stderr` do AprilTag | Durante vida do nó | duplica FD 2, pipe global e thread para suprimir uma mensagem da biblioteca C (`apriltag_detector.py:51-96`) | CPU baixa, mas custo arquitetural/risco de observabilidade alto | MÉDIA |

No repouso, o provável top 3 é: `controller_manager`, driver do braço/base e `move_group`; durante visão, AprilTag/câmera deve assumir a liderança. Isso é uma inferência, não medição.

## 3. Gambiarras / hacks

### G1 — redirecionamento global de `stderr`

- **Arquivo/linha:** `apriltag/apriltag/apriltag_detector.py:51-96`.
- **O que faz:** substitui o FD 2 do processo por um pipe e uma thread, filtrando uma frase emitida pelo código C.
- **Por que é gambiarra:** altera globalmente a saída de todas as bibliotecas do processo para contornar um warning específico de dependência.
- **Risco:** perda/atraso de logs, deadlock ou erro de teardown, mascaramento acidental e diagnóstico mais difícil.
- **CPU:** baixa em condição normal porque `os.read` bloqueia; não é o gargalo principal.
- **Prioridade:** ALTA para arquitetura/robustez.
- **Ação:** atualizar/corrigir `pupil_apriltags`, capturar o warning na fronteira nativa ou desabilitar `suppress_native_pose_warning`; remover o filtro após validar.

### G2 — serial bloqueante diretamente em callback ROS do braço

- **Arquivo/linha:** `so_arm_101_hardware/.../hardware_node.py:83-132`.
- **O que faz:** lê e escreve o barramento Feetech dentro do executor `SingleThreadedExecutor`, usando o mesmo lock.
- **Por que é gambiarra:** o tempo do hardware dita o tempo do executor; comando e leitura se bloqueiam mutuamente.
- **Risco:** starvation da leitura, jitter, comandos antigos e falsas falhas consecutivas.
- **Prioridade:** CRÍTICA.
- **Correção parcial feita:** QoS `KEEP_LAST(1)` nos dois lados, impedindo fila longa de comandos antigos.
- **Correção estrutural:** callback só atualiza `latest_command`; um único worker executa read/write a 20–30 Hz.

### G3 — seleção de interpretador/ambiente por prefixo e heurística

- **Locais:** launches do LiDAR, AprilTag e SO-101.
- **O que faz:** força `/usr/bin/python3` ou procura `VIRTUAL_ENV`/ancestrais do workspace para achar dependências.
- **Motivo aparente:** bibliotecas de hardware estão divididas entre Python do ROS e venv.
- **Risco:** comportamento diferente entre terminal, systemd e launch; dependências importadas da versão errada.
- **Prioridade:** ALTA.
- **Ação:** uma única instalação reproduzível para ARM, via rosdep + venv com `--system-site-packages`, e executáveis com shebang consistente.

### G4 — descoberta de serial via subprocesso `ls -l`

- **Arquivo/linha:** `base_hardware/.../portas.py:23-83`.
- **O que faz:** executa `ls`, interpreta texto e contém um `except` duplicado.
- **Risco:** dependência de formato/locale, topologia USB hardcoded, retorno `None` ambíguo.
- **CPU:** apenas startup, irrelevante no steady state.
- **Prioridade:** MÉDIA.
- **Ação:** usar `pathlib.Path('/dev/serial/by-path').iterdir()` como o driver novo do LiDAR já faz.

### G5 — supervisor dentro de supervisor

- **Arquivo/linha:** `MariolaZero/menuPrincipal/start.sh:1-7`.
- **O que faz:** relança Python para sempre, mesmo se o processo terminar corretamente.
- **Risco:** mascara exit code e cria política de restart concorrente com systemd.
- **Prioridade:** MÉDIA.
- **Ação:** `ExecStart` direto e `Restart=on-failure` com backoff no unit file.

### G6 — missão infinita e código operacional não empacotado

- **Arquivo/linha:** `pegar_e_colocar.py:55`; `so_arm_101_moveit_config/setup.py:13-22`.
- **O que faz:** repete a missão sem condição de saída, mas o script não é instalado como `console_script` nem em `data_files`.
- **Risco:** execução manual divergente da instalação; planejamento e visão infinitos.
- **Prioridade:** ALTA se usado; caso contrário classificar/remover.

## 4. Gargalos de performance

### P1 — visão faz trabalho demais por resultado útil

- **Local:** câmera 15 FPS (`camera/config/camera.yaml`), retificação no launch de produção e detector limitado a 10 Hz.
- **Causa:** RGB raw → RGB rectificado → mono; detecção e pose em cada frame recebido durante a sessão.
- **Custo atual:** ≈6,91 MB/s de payload RGB somando raw+rect, até 15 retificações e 10 detecções/s.
- **Efeito:** alto uso de CPU, largura de memória, alocações e interferência com control loop.
- **Correção feita:** detector limitado a 10 Hz antes de conversão/detecção.
- **Próximo teste:** câmera a 10 ou 15 FPS; formato mono/YUYV se suportado; `quad_decimate=1.5` ou `2.0`; medir precisão/distância.
- **Ganho esperado:** 50–70% do CPU do detector pela limitação já feita; outros 15–40% do caminho de imagem com FPS/formato, sem somar percentuais linearmente.

### P2 — ponte do braço e serial Feetech

- **Local:** `controllers.yaml:4`, `so101_system.cpp:148-162`, `hardware_node.py:117-132`.
- **Causa original:** `write()` publicava sempre e a callback executava `send_action()` imediatamente.
- **Correção feita:** QoS depth 1, callback latest-value, ciclo serial único a 30 Hz, deduplicação e heartbeat de 5 Hz.
- **Ganho esperado:** redução de até 40% no número máximo de escritas ao alinhar 50→30 Hz; muito mais em repouso com deduplicação.

### P3 — base faz pelo menos seis operações de protocolo por ciclo

- **Local:** `hardware_node.py:127-144`, `mariola_adapter.py:162-208`, `controleMotores.py:116-219,278-365`.
- **Causa:** 30 ciclos/s, leitura do brick + duas expansões e escrita do brick + duas expansões; novos futures em cada leitura/escrita.
- **Efeito:** aproximadamente 180 operações de protocolo/s, mesmo com zero constante.
- **Correção feita:** alvos são comparados após quantização; mudança/zero são imediatos e repetições usam heartbeat de 5 Hz.
- **Próxima ação:** medir p50/p95 de read e write e validar parada no hardware.

### P4 — taxas ROS/control/hardware desalinhadas

- **Local atual:** controle, base e braço alinhados em 30 Hz.
- **Ação:** teste A/B com `controller_update_rate:=50` para confirmar estabilidade do mecanum e do trajectory controller.
- **Ganho esperado:** até 40% do overhead específico do control loop/publicação, não do processo inteiro.

### P5 — componentes caros sobem sem demanda

- **Local:** LiDAR, câmera/AprilTag e `move_group` no launch principal (`robot.launch.py:37-51,79-115`).
- **Causa:** launch monolítico; câmera é desligada depois por serviço, mas nós continuam residentes; LiDAR continua girando/lendo; MoveIt continua residente.
- **Ação:** launch arguments/lifecycle para `use_lidar`, `use_vision`, `use_manipulation`; perfil mínimo inicia apenas controle e RSP.
- **Ganho esperado:** todo o custo do componente desativado e menos competição no startup.

### P6 — polling vazio do LiDAR

- **Local:** `lidar/lidar/lidar_node.py:46,72-90`.
- **Antes:** timer 100 Hz para um scan que aparece aproximadamente uma vez por rotação (~5 Hz).
- **Depois:** 20 Hz.
- **Ganho:** ~80 callbacks vazias/s removidas; impacto global pequeno, mas risco quase zero.

## 5. Código morto, obsoleto ou possivelmente não utilizado

| Item | Classificação | Evidência | Ação |
|---|---|---|---|
| `camera/config/camera_info.backup.yaml` | REMOVER | não referenciado e instalado pelo glob | excluir após confirmar que não é artefato de calibração necessário |
| `so101_follower_antigo.json` | REMOVER | configuração antiga, sem referência, instalada | arquivar fora do pacote ou excluir |
| `build/install` de `navigation` e `motor_control` | REMOVER (gerado) | índices instalados existem, fonte atual não | reconstruir workspace limpo em diretório novo; não confiar no overlay atual |
| `pegar_e_colocar.py` | POSSIVELMENTE NÃO UTILIZADO | não é console script nem instalado; só testes importam módulos relacionados | instalar como executável suportado ou remover/documentar como ferramenta manual |
| `/scan_front` | POSSIVELMENTE NÃO UTILIZADO | nenhum subscriber no fonte atual | verificar grafo real e clientes externos; então tornar LiDAR opcional |
| quatro tópicos contínuos AprilTag | POSSIVELMENTE NÃO UTILIZADOS externamente | ação guarda resultados internamente; nenhum subscriber versionado | parâmetros para publicar apenas os produtos exigidos |
| API extensa em `motores.py`/`placaControleMotor.py` | POSSIVELMENTE NÃO UTILIZADA | runtime ROS usa pequeno subconjunto; pode ser API pública legado | medir cobertura e separar pacote `legacy`, sem remoção cega |
| cópias de drivers em vários exemplos `MariolaZero` | LEGADO/DUPLICADO | implementações repetidas fora do runtime ROS | manter exemplos fora da imagem de produção e apontar para uma biblioteca única |
| parâmetros `state_publish_rate` do JTC | POSSIVELMENTE IGNORADOS | não aparecem na API gerada do JTC ROS Jazzy instalada localmente | confirmar `ros2 param describe`; remover ou substituir pelo parâmetro suportado |
| `tests_require` nos `setup.py` | OBSOLETO | setuptools emite warning | usar dependências de teste em `package.xml`/ambiente |

Não remover o `fancontrol.py`: o loop dorme entre leituras, evita sysfs writes desnecessários e é proteção térmica (`fancontrol.py:118-134`).

## 6. Comunicações ROS 2

Taxas são as configuradas/deduzidas (**ESTÁTICA**); `hz`/`bw` reais devem ser medidos.

| Tópico/serviço/action | Publisher → subscriber | Taxa/payload esperado | QoS/observação | Avaliação |
|---|---|---|---|---|
| `/so101_hardware/raw_joint_states` | driver Python → `SO101System` | 30 Hz, 6 juntas | reliable, depth 1 — CORRIGIDO | adequado para latest state |
| `/so101_hardware/command_positions` | `SO101System` → driver Python | 30 Hz máximo, 6 doubles | reliable, depth 1 | callback guarda apenas o alvo mais recente |
| `/base_hardware/raw_joint_states` | base Python → `MariolaSystem` | 30 Hz, 4 rodas | reliable, depth 1 | adequado |
| `/base_hardware/command_velocities` | `MariolaSystem` → base Python | até 30 Hz, 4 velocidades | reliable, depth 1 | escrita física deduplicada após quantização |
| `/joint_states` | broadcaster → RSP/MoveIt/clientes | provável 30 Hz, 10 juntas | verificar em runtime | alinhado ao hardware |
| `/arm_controller/controller_state` | JTC → observadores | provável update/control rate | verificar parâmetro real | possivelmente superpublicado |
| `/gripper_controller/controller_state` | JTC → observadores | provável update/control rate | verificar | possivelmente superpublicado |
| `/cmd_vel` | cliente externo → mecanum | orientada a evento | timeout 0,25 s | correto para segurança |
| `/odom` | mecanum → consumidores | provável 30 Hz | verificar QoS | alinhado ao controle |
| `/tf` odom→base | mecanum → TF | provável 30 Hz | dinâmico | alinhado ao controle |
| `/tf`, `/tf_static` do robô | RSP → TF | joints / fixos | padrão ROS | não foi encontrada TF duplicada |
| `/camera/image_raw` | `usb_cam` → `image_proc` | 15 FPS, 230.400 B/frame RGB | sensor/best effort esperado | ~3,46 MB/s durante captura |
| `/camera/image_rect` | `image_proc` → AprilTag | 15 FPS, mesmo tamanho | detector best effort depth 1 | ~3,46 MB/s; 10 FPS detectados |
| `/camera/camera_info` | câmera → AprilTag | tipicamente junto aos frames | sensor profile | pequeno |
| `~/detect` AprilTag action | cliente → detector | sob demanda, feedback 5 Hz | action | bom contrato de demanda |
| `/apriltags/poses_camera` | detector → externo | até 10 Hz | reliable depth 1 — CORRIGIDO | desligar se ninguém usa |
| `/apriltags/poses` | detector → externo | até 10 Hz | reliable depth 1 — CORRIGIDO | desligar se ninguém usa |
| `/apriltags/detections_camera` | detector → externo | até 10 Hz | reliable depth 1 — CORRIGIDO | dados redundantes para ação interna |
| `/apriltags/detections` | detector → externo | até 10 Hz | reliable depth 1 — CORRIGIDO | idem |
| TF camera→tag | detector → TF | até 10 Hz/tag | dinâmica | semanticamente correta |
| `/camera/set_capture` | detector → `usb_cam` | início/fim de sessão/retry | serviço | boa estratégia; confirmar driver 0.8.x |
| `/scan_front` | LiDAR → consumidor externo | ~5 Hz, 121 ranges | sensor best effort | nenhum consumidor versionado |

As duas pontes de hardware são justificáveis para isolar bibliotecas Python, porém somam quatro tópicos internos, serialização DDS e scheduling. Composição/intra-process não atravessa Python/C++ desta forma. O ganho mais seguro é reduzir taxas/filas; reescrever driver como plugin direto só faz sentido se profiling mostrar DDS relevante frente ao custo serial.

## 7. Timers

| Nó/local | Timer | Frequência atual | Problema | Recomendação |
|---|---|---:|---|---|
| `controller_manager` | update loop | 30 Hz | alinhado aos drivers | validar no hardware |
| SO-101 Python | ciclo write/read | 30 Hz | escrita por mudança/heartbeat 5 Hz | validar latência e CPU |
| base Python | read 30 Hz + write variável | 30/5 Hz em repouso | leitura ainda usa três transações | medir p50/p95 |
| LiDAR ROS | `take_scan()` | 20 Hz | era 100 Hz para ~5 scans/s | CORRIGIDO; 10 Hz também pode bastar |
| AprilTag | câmera idle | 4 Hz (0,25 s) | baixo custo | manter |
| AprilTag | feedback | 5 Hz | só sessão ativa | manter |
| readiness hardware | verificação | 10 Hz | transitório, encerra | manter |
| readiness controller | polling serviço | 10 Hz | transitório, encerra | manter |
| teleop teclado | polling | 50 Hz | notebook apenas; exagerado | 20 Hz ou leitura bloqueante de terminal |
| teleop grafo | serviços | 2 Hz | notebook apenas | aceitável |
| menu Mariola | teclado/tela | 10 Hz | mais loops busy ao segurar botão | eventos/sleep curto nos loops internos |
| fan control | temperatura | a cada 3 s | segurança térmica | manter |
| simulação | control loop | 250 Hz | somente desenvolvimento | nunca usar na Banana física |

## 8. Todos os loops suspeitos

| Loop | Estado | Bloqueio/sleep | Risco |
|---|---|---|---|
| `LidarDriver._read_loop` (`lidar_driver.py:267-280`) | runtime ativo | serial timeout 0,1 s + Event 1 ms quando vazio | não é busy loop; parse byte a byte consome CPU proporcional ao fluxo |
| `NativeWarningFilter._run` (`apriltag_detector.py:65-86`) | runtime ativo | `os.read` bloqueante | não é busy loop, mas hack global |
| espera da action AprilTag | sessão ativa | sleep ~20 ms | não é busy; uma thread daemon por goal e concorrência de goals merecem limite |
| espera de captura de câmera | transição | Condition/intervalo ~20 ms, timeout 5 s | aceitável |
| settle do MoveIt | movimento | `spin_once(timeout=0.05)` | não é busy; barreira real, melhor que sleep fixo |
| `PegarEColocar.executar` (`pegar_e_colocar.py:55`) | manual/possível | ações bloqueiam | não é busy, mas repete carga pesada sem fim |
| menu principal externo (`start.py:31`) | se serviço habilitado | sleep 0,1 s | polling moderado |
| waits de botão (`start.py:47-54`) | se botão voltar pressionado | sleep 10 ms — CORRIGIDO | antes era busy loop real e podia ocupar um core |
| `start.sh:4-7` | se serviço habilitado | sleep 1 s após processo | não é busy; política de restart ruim |
| `fancontrol.py:118-128` | serviço necessário | sleep 3 s | eficiente e necessário |
| giroscópio legado | somente se instanciado | sleep 5 ms apesar de comentário “25 ms” | ~200 Hz; corrigir comentário/taxa se usado |
| sensor de reflexão legado | somente se instanciado | sleep 10 ms | 100 Hz; validar necessidade |
| mux VL53 legado | somente se instanciado | sleep ~100 ms | aceitável |
| exemplos de câmera/IA/Bluetooth/sensores | não lançados em produção | variável | muitos loops infinitos; excluir da imagem/serviços, não executar junto ao ROS |

Não foi encontrado busy loop verdadeiro no caminho ROS principal. Os dois busy waits confirmados no menu legado foram corrigidos com polling de 10 ms.

## 9. Arquitetura atual

```text
                         notebook / clientes externos
                    cmd_vel, actions MoveIt, telemetria
                                  |
                                  v
+-----------------------------------------------------------------------+
| Banana Pi — robot.launch.py                                           |
|                                                                       |
|  usb_cam --RGB30--> image_proc --RGB rect30--> AprilTag(action)       |
|       ^ set_capture                         | poses/detections/TF      |
|                                             |                         |
|  LiDAR thread/serial --~5Hz--> /scan_front  |                         |
|                                                                       |
|  SO101 Python <--ROS 30Hz--> SO101System -----+                       |
|       | Feetech serial                        |                       |
|                                               v                       |
|  Base Python  <--ROS 30Hz--> MariolaSystem --> controller_manager     |
|       | brick + serial expansão              | 50Hz                  |
|                                               +--> joint_states       |
|                                               +--> odom / TF           |
|                                               +--> arm/base control    |
|                                                                       |
|  robot_state_publisher <--- joint_states                              |
|  move_group <----------- joint_states / controllers / TF              |
+-----------------------------------------------------------------------+

Possível processo paralelo fora do launch: menuPrincipal + fancontrol.
```

Pontos de contenção:

- `controller_manager` produz comando mais rápido do que os drivers atualizam estado;
- no braço, a serial bloqueia callbacks e compartilha lock entre leitura/escrita;
- visão disputa CPU e memória durante a janela em que precisão de controle também importa;
- launch monolítico inicia periféricos antes da confirmação de que hardware de movimento está pronto.

## 10. Arquitetura sugerida

```text
Perfil core (sempre):
  robot_state_publisher
  controller_manager @ 30Hz (a validar)
      |-- plugin SO101 -- latest value --> worker Python único @ 30Hz --> serial
      `-- plugin base  -- latest value --> ciclo I/O @ 20–30Hz --> buses
  fancontrol

Perfis sob demanda:
  navigation: habilita relé/thread LiDAR e consumidores de /scan_front
  vision:     habilita captura 10–15 FPS -> rect/mono -> AprilTag <=10Hz
  manipulate: inicia move_group; executa missão finita/action; encerra ou fica warm

Notebook:
  RViz, teleop, visualização MoveIt, rosbag, profiling/dashboard
```

Recomendações arquiteturais:

1. Dividir `robot.launch.py` em `core.launch.py` e grupos condicionais `navigation`, `vision`, `manipulation`.
2. Manter ownership de cada barramento em exatamente uma thread/processo. Callback só troca estado latest-value.
3. Usar actions para tarefas finitas, com cancelamento, timeout e resultado; eliminar o `while True` da tarefa.
4. Definir um “performance budget” por modo e verificar CPU, RSS, temperatura e deadlines em CI de hardware.
5. Preservar fail-safe: timeout de `/cmd_vel`, stop explícito, fancontrol e shutdown coordenado.

## 11. O que deve rodar onde

### Banana Pi

- drivers físicos do braço, base, câmera e LiDAR apenas quando necessários;
- `controller_manager`, controladores e `robot_state_publisher`;
- TF e odometria essenciais;
- AprilTag apenas na janela de detecção;
- `move_group` somente se a manipulação precisa continuar autônoma sem notebook;
- fancontrol e monitor leve de saúde/temperatura.

### Notebook

- RViz, Gazebo, `joint_state_publisher_gui`, teleop e dashboards;
- rosbag/gravação de imagens e scans;
- profiling e visualização de árvore TF;
- MoveIt/RViz visual e, se a rede for confiável e perda do notebook for tratada com segurança, planejamento de alto nível;
- desenvolvimento, calibração e exemplos.

### Não deve coexistir com produção na Banana

- exemplos `MariolaZero`, Gazebo/RViz, testes, build jobs;
- resíduos de pacotes removidos no overlay;
- menu principal, se display/teclado não forem requisito operacional. Se forem, corrigir busy waits e integrá-lo ao supervisor único.

## 12. Plano de correções por fases

### Fase 0 — baseline e segurança (antes de novo deploy)

1. Resolver as três falhas da suíte com medidas físicas confirmadas.
2. Fixar uma imagem ARM reproduzível e garantir `pyserial`, `pupil-apriltags`, LeRobot e ROS no mesmo ambiente de execução.
3. Fazer build em workspace limpo, sem reutilizar `install/` que contém pacotes órfãos.
4. Capturar baseline dos quatro cenários: repouso, base móvel, braço móvel, AprilTag ativo.

### Fase 1 — quick wins de baixo risco

1. **Já feito:** AprilTag 30→máx. 10 detecções/s.
2. **Já feito:** QoS latest-value depth 1 na ponte do braço e saídas AprilTag.
3. **Já feito:** polling LiDAR 100→20 Hz.
4. **Já feito:** remover `sleep(1)` após estado de detecção; a função já aguarda assentamento real.
5. Tornar LiDAR opcional e desligado quando o grafo real confirmar ausência de consumidor.
6. **Já feito:** corrigir os dois busy waits do menu com `sleep(0.01)`; eventos de GPIO ainda seriam a solução ideal.

### Fase 2 — controle e I/O

1. Implementar worker único latest-value do SO-101 a 30 Hz.
2. Testar `controller_manager.update_rate=30` e taxas de estado/odom/TF de 20–30 Hz.
3. Instrumentar duração de I/O da base e braço; registrar p50/p95/p99 e deadline misses.
4. Deduplicar comando estacionário com heartbeat, após confirmar watchdog/protocolo.

### Fase 3 — componentes sob demanda

1. Separar launch core/vision/navigation/manipulation.
2. Câmera 10–15 FPS enquanto detecta; testar mono e `quad_decimate`.
3. `move_group` por demanda ou mantido warm somente quando missão de manipulação estiver armada.
4. Converter pick-and-place em action finita/cancelável.

### Fase 4 — limpeza e consolidação

1. Remover backups/configs antigos e overlays órfãos por rebuild limpo.
2. Consolidar drivers duplicados de `MariolaZero`.
3. Corrigir metadados/dependências diretas em `package.xml` e remover `tests_require`.
4. Substituir heurísticas de Python/serial por configuração explícita e udev aliases.

## 13. Patches concretos e profiling

### Patch aplicado A — limitar AprilTag antes do trabalho caro

Antes:

```python
def image_callback(self, message):
    image = self.image_to_mono8(message)
    detections = self.detector.detect(image, estimate_tag_pose=True, ...)
```

Depois:

```python
self.declare_parameter('max_detection_rate_hz', 10.0)
self.detection_period = 1.0 / detection_rate

now = time.monotonic()
if now - self.last_detection_time < self.detection_period:
    return
self.last_detection_time = now
image = self.image_to_mono8(message)
detections = self.detector.detect(image, estimate_tag_pose=True, ...)
```

### Patch aplicado B — não acumular estado/comando velho

Antes: QoS reliable com profundidade 10.
Depois, Python e C++:

```python
QoSProfile(history=KEEP_LAST, depth=1, reliability=RELIABLE)
```

```cpp
const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
```

### Patch aplicado C — reduzir callbacks vazias do LiDAR

```diff
- poll_rate_hz: 100.0
+ poll_rate_hz: 20.0
```

### Patch aplicado D — remover espera temporal falsa

```diff
  mover_para_estado(... "detect_apriltags" ...)
- sleep(1)
  obter_pose_da_april_tag(...)
```

`mover_para_estado` já retorna após a execução e o assentamento verificado; o segundo extra apenas atrasava cada ciclo.

### Patch aplicado E — eliminar busy wait do botão do menu

```diff
 while botao_voltar_esta_pressionado:
-    pass
+    sleep(0.01)
```

### Patch proposto F — worker latest-value do braço

Alteração concreta:

```python
def _command_callback(self, msg):
    validate(msg)
    with self._command_lock:
        self._latest_command = tuple(msg.data)

def _io_cycle(self):               # único owner da serial, 30 Hz
    observation = follower.get_observation()
    command = atomic_latest_command()
    if command_changed or heartbeat_due:
        follower.send_action(command)
    publish(observation)
```

Não usar dois callbacks reentrantes para a mesma serial; mais threads não eliminam a contenção física.

### Patch proposto G — launch por perfis

Adicionar argumentos `use_lidar`, `use_vision`, `use_manipulation` e `IfCondition`/`GroupAction`. Defaults de produção devem refletir a missão real; em `core`, os três são `false`. Iniciar periféricos somente após readiness do controle também reduz contenção no boot.

### Patch proposto H — base com deduplicação segura

Guardar `last_sent_command` e `last_send_time`; enviar quando o comando mudar ou quando vencer `command_heartbeat_sec`. O timeout de comando continua impondo zero e a transição para zero sempre deve ser transmitida imediatamente.

### Roteiro de profiling na Banana

Executar por 60 s em cada cenário, com mesma alimentação e ventilação:

```bash
pidstat -durwt -p ALL 1
ps -eLo pid,tid,psr,pcpu,pmem,nlwp,comm,args --sort=-pcpu
ros2 topic hz /joint_states /odom /scan_front /camera/image_rect
ros2 topic bw /camera/image_raw /camera/image_rect /scan_front
ros2 topic info -v /scan_front
perf stat -p PID -e task-clock,cycles,instructions,cache-misses,context-switches -I 1000
```

Registrar também temperatura e frequência para detectar throttling:

```bash
watch -n 1 'cat /sys/class/thermal/thermal_zone*/temp; cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq'
```

Cenários: (A) core parado; (B) base em trajetória repetível; (C) braço em trajetória repetível; (D) action AprilTag 30 s; (E) missão completa. Comparar baseline, quick wins, 30 Hz de controle, câmera 15/10 FPS e componentes opcionais. Medir latência de comando e misses, não só média de CPU.

Validação local desta revisão:

- build completo dos 17 pacotes do workspace: concluído;
- testes direcionados ampliados: 64 aprovados e 1 falha pré-existente de tamanho do cubo;
- suíte completa: 158 aprovados, 3 falhas pré-existentes;
- `git diff --check`: limpo;
- profiling ROS não foi possível neste host x86 sem processos/hardware e com descoberta DDS bloqueada pelo sandbox. Portanto nenhum número de `%CPU` deste relatório é apresentado como medição.

## 14. Resultados esperados e respostas diretas

1. **Qual nó provavelmente mais consome CPU?** Durante visão, `apriltag_detector` junto de `usb_cam/image_proc`. Em repouso, provavelmente `controller_manager` + drivers de hardware; `move_group` deve ser medido separadamente.
2. **Há processamento desnecessário?** Ainda há: LiDAR sem consumidor versionado, cinco frames retificados por segundo que não chegam ao detector, quatro saídas AprilTag sem consumidor local e MoveIt sempre residente. LiDAR e MoveIt foram mantidos por decisão operacional.
3. **Há timers/frequências exagerados?** Os principais foram corrigidos: LiDAR 100→20 Hz e controle 50→30 Hz. Teleop 50 Hz fica no notebook e sensores legados podem alcançar 100–200 Hz se ativados.
4. **Quais são as piores gambiarras?** Callback serial bloqueante do braço, filtro global de `stderr`, ambientes Python/portas descobertos por heurística, missão infinita não empacotada e supervisor/busy waits do menu.
5. **O que pode sair da Banana Pi?** RViz/Gazebo/teleop já devem ficar fora; também rosbag, dashboards e visualização MoveIt. LiDAR, MoveIt e menu podem ser desligados por perfil quando a missão não precisar deles.
6. **O que deve iniciar apenas sob demanda?** Captura/detecção visual, LiDAR e `move_group`/tarefa de manipulação.
7. **Há busy loops?** Não no runtime ROS principal. Havia dois no menu legado ao segurar o botão voltar; ambos foram corrigidos com sleep de 10 ms.
8. **Há processamento duplicado?** Sim: raw RGB → rect RGB → mono, pontes ROS para ambos hardwares, escritas idênticas repetidas e muitas cópias divergentes dos drivers nos exemplos.
9. **Há publicação ROS excessiva?** Foi reduzida para 30 Hz em comando, joint state, odom/TF; os quatro tópicos AprilTag permanecem. Confirmar taxas reais com `ros2 topic hz`.
10. **A imagem é processada demais?** O pipeline caiu de 30 para 15 FPS e o detector fica em 10 Hz. O custo continua relevante, mas preserva retificação e `quad_decimate=1.0` para precisão.
11. **Callbacks bloqueiam?** Sim: leitura/escrita do braço e ciclo completo da base; AprilTag bloqueia seu executor durante cada detecção. QoS depth 1 evita backlog de imagem/comando, não elimina jitter.
12. **Existe código morto/obsoleto?** Sim: configs antigas/backups, overlays de pacotes ausentes, script operacional não instalado, parâmetros possivelmente ignorados e cópias legadas.
13. **Quais cinco mudanças fazer primeiro?** A lista em destaque abaixo.

## AS 5 PRIMEIRAS MUDANÇAS PARA REDUZIR CPU

1. **Medir e limitar visão:** manter AprilTag em 10 Hz (já aplicado), testar câmera em 10–15 FPS e `quad_decimate=1.5/2.0` com critério de precisão.
2. **Reestruturar o braço para latest-value + worker serial único a 30 Hz:** depth 1 já aplicado; esta é a correção mais importante para latência e jitter.
3. **Desligar o LiDAR quando não houver consumidor:** tornar `use_lidar` condicional; o polling 100→20 Hz já foi aplicado.
4. **Subir `move_group` e a tarefa somente por demanda e tornar a missão finita/cancelável:** remove CPU/RSS ocioso e picos infinitos.
5. **Alinhar controle/estado/odom em 30 Hz e deduplicar escritas estacionárias com heartbeat:** aplicar apenas depois do A/B no hardware.

Meta realista a validar: no cenário de visão, a mudança já aplicada deve reduzir aproximadamente dois terços do custo do detector; no repouso, perfis sob demanda e deduplicação podem produzir ganho maior que micro-otimizações Python. O critério de aceite deve combinar CPU total, p95 de latência, perda de deadline, temperatura, estabilidade da base e precisão AprilTag.
