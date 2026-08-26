# Auditoria do bringup ROS 2 físico

Data da auditoria: 2026-08-26. Alvo: Banana Pi, ROS 2 Jazzy.

## Escopo e método

Foram inspecionados os packages em `src/cbr_work`, os launches, YAMLs, URDF/Xacro,
plugins `ros2_control`, drivers Python, interfaces, testes e logs reais em
`~/.ros/log`. O código foi compilado, os launches foram expandidos com
`--show-args`, o URDF composto foi validado com `check_urdf` e a política de
falha dos sensores foi exercitada sem hardware.

Este computador não possui `/dev/serial/by-path`, `/dev/video1`,
`/dev/gpiochip1` nem `/dev/ttyS2`. Portanto, CPU, RSS, temperatura, latência
serial e Hz efetivos **não foram medidos no hardware final**. Valores derivados
de configuração são indicados como tal; nenhum número foi inventado.

## A. Diagnóstico da arquitetura auditada

### Fluxo de launch anterior

```text
robot.launch.py
├── hardware.launch.py
│   ├── so101_hardware_node                    [se arm]
│   ├── base_hardware_node                     [se base]
│   ├── robot_state_publisher                  [único]
│   ├── hardware_readiness                     [transitório]
│   └── ros2_control_node + 4 spawners         [após estado físico]
├── sensors.launch.py
│   ├── lidar_node
│   └── imu_node
├── localization.launch.py
│   └── ekf_filter_node
├── perception.launch.py
│   ├── /camera/driver
│   ├── /camera/rectify
│   └── apriltag_detector
└── manipulation.launch.py
    └── move_group
```

Os módulos já existiam, mas o ponto de entrada não expressava um perfil de
produção, qualquer falha de sensor/percepção/EKF/MoveIt podia emitir `Shutdown`
global, os sensores eram forçados a `/usr/bin/python3` e `move_group` concorria
com a ativação dos controllers.

### Nós e comunicação principal

| Produtor | Interface | Consumidor | Papel/parâmetros principais |
|---|---|---|---|
| Nav2/teleop | `/cmd_vel` (`TwistStamped`) | `base_controller` | remap de `~/reference`; timeout 0,25 s |
| `base_controller` | command interfaces de 4 rodas | `MariolaSystem` | mecanum, raio 0,034 m, braço cinemático 0,2225 m |
| `MariolaSystem` | `/base_hardware/command_velocities` (`WheelCommand`) | `base_hardware_node` | rad/s, saturação conjunta em 7 rad/s |
| `base_hardware_node` | barramento do brick + serial 250000 | controladores físicos | converte rad/s para comando discreto, sinais e ticks calibrados |
| encoders/placas | posição/velocidade | `base_hardware_node` | 1644/3288 ticks por volta conforme o barramento |
| `base_hardware_node` | `/base_hardware/raw_joint_states` | `MariolaSystem` | estado mais recente das 4 rodas |
| `base_controller` | `/wheel/odom` | EKF | velocidade planar; TF própria desabilitada |
| `imu_node` | `/imu/data` | EKF | gyro Z; serial 115200 em `/dev/ttyS2` |
| EKF | `/odom`, `odom -> base_footprint` | Nav2/RSP/consumidores | 20 Hz, filas 2, único dono dessa TF |
| `ros2_control` | `/joint_states` | RSP, MoveIt, teleop | 10 juntas, alvo de 30 Hz |
| MoveIt | `FollowJointTrajectory` | arm/gripper controllers | ações padrão dos JTCs |
| `SO101System` | `/so101_hardware/command_positions` | driver SO-101 | radianos/m no limite ROS |
| driver SO-101 | LeRobot/Feetech serial | servos | conversão ROS ↔ convenção LeRobot; grau só na fronteira configurável |
| servos | observação | `/so101_hardware/raw_joint_states` | posição e velocidade calculada, 30 Hz |
| LiDAR | `/scan_front` | AMCL/costmaps, quando lançados externamente | `LaserScan`, `lidar_front_link`, aproximadamente 5 scans/s |
| câmera | `/camera/image_raw`, `camera_info` | `image_proc` | 320×240, MJPEG, 15 FPS |
| rectify | `/camera/image_rect` | AprilTag | remap local `image -> image_raw` |
| AprilTag | `apriltags/analyze` (action), poses/detections e TFs de tags | missão | até 10 Hz; captura sob demanda |

Serviços explícitos relevantes: `/so101_hardware/calibrate` (`Trigger`),
`/controller_manager/list_controllers` e serviços padrão do controller manager.
O teleop Xbox também usa serviços de cancelamento de goals Nav2. Não há
namespace global do robô; somente a câmera usa `/camera`.

### TF

`check_urdf` aceitou o Xacro composto. A árvore estrutural é:

```text
odom --EKF(dynamic)--> base_footprint
  `--RSP(fixed)--> base_link
      |-- rodas
      |-- imu_link
      |-- lidar_front_link
      `-- upper_platform_link --> arm_base_link --> cadeia do braço
                                                `--> camera_optical_frame
```

`robot_state_publisher` é único. `base_controller.enable_odom_tf=false` evita
dois publicadores de `odom -> base_footprint`. Tags são TFs dinâmicas porque
só existem enquanto observadas. `map -> odom` só aparece quando AMCL/SLAM é
iniciado; o repositório não oferece launch integrado para isso.

### Nav2 e SLAM

Há parâmetros em `tools/amcl_localization.yaml`, `tools/nav2_navigation.yaml` e
`tools/slam_mapping.yaml`, mas não há package/launch que os componha ao bringup.
Logs antigos mostram Nav2 iniciado manualmente e timeouts por ausência de TF
`odom`. Logo, navegação não pode ser classificada como subsistema funcional do
bringup atual. Ela foi deliberadamente não “inventada” nesta refatoração.

## B. Problemas encontrados

| Severidade | Problema | Situação |
|---|---|---|
| **CRÍTICO** | Falha de um sensor emitia `Shutdown` e derrubava inclusive drivers de movimento | corrigido com política por missão |
| **CRÍTICO** | Ambientes Python divergentes: `/usr/bin/python3` tinha `rclpy`, mas não `serial`, `gpiod`, `pupil_apriltags` ou `lerobot` | seleção coerente implementada; imagem ainda precisa instalar `gpiod` |
| **CRÍTICO** | Drivers/controladores podiam ativar antes de receber estado físico válido | já havia barreira por mensagem; preservada e validada |
| **ALTO** | MoveIt iniciava antes de `joint_state_broadcaster`, `arm_controller` e `gripper_controller` estarem `active` | corrigido com readiness via serviço |
| **ALTO** | Artefatos antigos de `interfaces` expunham actions apagadas e causavam símbolo indefinido ao importar `WheelCommand` | build/install desse package limpos e reconstruídos |
| **ALTO** | Nav2/AMCL/SLAM são configurações soltas, sem lifecycle/orquestração versionada | pendente por falta de requisito de missão/mapa |
| **ALTO** | Descoberta de portas usa `ls /dev/serial/by-path`, caminhos de plataforma e tratamento duplicado | pendente de teste no hardware |
| **ALTO** | Seis testes contradizem geometria/calibração/configuração corrente | não alterado sem medições físicas |
| **MÉDIO** | IMU fazia 50 transações/s para EKF de 20 Hz | reduzida para 20 Hz |
| **MÉDIO** | LiDAR contínuo sem consumidor versionado | desligado no perfil canônico; opt-in |
| **MÉDIO** | Percepção USB/AprilTag era parte do perfil completo e concentrava CPU | opt-in; 15 FPS/10 Hz |
| **MÉDIO** | Filtro AprilTag redireciona globalmente `stderr` por FD/thread | permanece; biblioteca deve ser corrigida/substituída antes de remover |
| **MÉDIO** | Parâmetro de docking Nav2 está em 50 Hz, acima do pipeline de 20 Hz | reduzir para 20 Hz se docking for realmente utilizado |
| **BAIXO** | Arquivos de backup/legado e configs de simulação a 250 Hz confundem produção | não são incluídos pelo novo bringup |

## C. Causa do abort atual

Evidência do log `2026-08-20-16-07-47-133035-evandro-282808`:

1. `lidar_node` terminou primeiro com código 1: `O módulo Python gpiod não está instalado`.
2. O `OnProcessExit` do launch tratou o LiDAR como fatal e emitiu `Shutdown`.
3. O launch enviou `SIGINT` a todos os demais processos.

**Erro raiz:** ambiente Python inconsistente, concretizado pela falta de
`gpiod`, combinado com uma política errada que tornava um sensor não essencial
fatal para todo o robô.

**Erros secundários simultâneos:** IMU sem módulo `serial`; base sem
`/dev/serial/by-path`; câmera terminando com `-6` durante a cascata de shutdown.
Logs de `image_proc` com `-11` encontrados em outras sessões ocorreram depois
de Ctrl-C e não sustentam a hipótese de causa raiz. Logs ainda mais antigos
registram contenção/ausência de resposta no SO-101; isso é um risco físico
separado, não a origem comprovada desta ocorrência.

## D. Gambiarras e dívida técnica

| Arquivo/local | Problema | Motivo provável | Classificação/risco | Solução |
|---|---|---|---|---|
| `bringup/launch/sensors.launch.py` antigo | `prefix=/usr/bin/python3` | obter `rclpy` do sistema | perigosa: dependências sumiam | **removida**; `CBR_SENSOR_PYTHON`/venv coerente |
| `base_hardware/portas.py` | subprocesso `ls`, caminhos fixos e exceções duplicadas | suportar várias revisões da placa | temporária, risco alto | enumerar dispositivos com `pyudev`, VID/PID/serial e erro diagnóstico |
| `apriltag_detector.py`, filtro de `stderr` | troca FD 2 global e cria thread | suprimir warning de biblioteca C | aceitável temporariamente, risco médio | corrigir/upstream ou configurar logging nativo |
| `tools/*.yaml` | configuração operacional fora de package e BT com caminho absoluto | protótipo manual de Nav2 | desnecessária no produto, risco alto | criar package Nav2 só após validar frames/mapa/plugins |
| `build/interfaces`, `install/interfaces` | ABI antiga de actions removidas | build incremental após mudança de IDL | perigosa | **corrigida** com rebuild limpo do package |
| drivers legados de motor | sleeps curtos e timeout de 30 s na calibração | protocolo/tempo físico do firmware | necessária até prova contrária | instrumentar protocolo; não remover cegamente |
| MoveIt antigo | início concorrente aos spawners | tentativa de paralelizar startup | desnecessária, risco alto | **removida** com readiness determinístico |
| perfil antigo | câmera/LiDAR sempre residentes | noção de “sistema completo” | desperdício médio/alto | **corrigido** por opt-in |

Não foi encontrado `TimerAction` no novo caminho. Os timers de readiness são
polling não bloqueante de curta duração; terminam ao atingir a condição. Sleeps
de protocolo não foram removidos sem documentação do firmware.

## E. Frequências, timers e QoS

| Componente | Antes/configuração | Agora/recomendado | Avaliação |
|---|---:|---:|---|
| `controller_manager` | 30 Hz | 30 Hz | coerente com ambos os drivers |
| base: read/ciclo | 30 Hz | 30 Hz | necessário para odometria e controle |
| base: write idêntico | até 30 Hz | mudança imediata + heartbeat 5 Hz | evita tráfego repetido |
| SO-101: read/ciclo | 30 Hz | 30 Hz | coerente com JTC |
| SO-101: write idêntico | até 30 Hz | mudança imediata + heartbeat 5 Hz | reduz serial em repouso |
| `/joint_states`, JTC | alvo 30 Hz | 30 Hz | validar Hz efetivo |
| IMU | **50 Hz** | **20 Hz** | alinhada ao EKF; 60% menos transações |
| EKF | 20 Hz | 20 Hz | adequado ao pipeline móvel |
| LiDAR físico | ~5 Hz | ~5 Hz | limite do sensor |
| timer LiDAR | 20 Hz | 10–20 Hz, sob demanda | 20 é aceitável; perfil padrão desliga |
| câmera | 15 FPS | 10–15 FPS | 15 apenas durante missão visual |
| AprilTag | máximo 10 Hz | 5–10 Hz | 10 para aproximação; uma thread nativa |
| Nav2 controller/smoother | 20/20 Hz | 20/20 Hz | coerente com `/cmd_vel` e base |
| costmap local | update 5, publish 2 Hz | manter | adequado |
| costmap global | 1/1 Hz | manter | adequado |
| planner | esperado 0,5 Hz | manter | adequado |
| docking | 50 Hz | 20 Hz se habilitado | 50 Hz não agrega ao atuador de 30 Hz |
| teleop Xbox | 20 Hz | 20 Hz | watchdog em 0,30 s |
| teleop teclado | 50 Hz | 20 Hz ou leitura orientada a evento | workstation apenas |
| readiness hardware/controller | 10/5 Hz | manter | transitórios, sem busy wait |

QoS auditado:

- pontes C++ ↔ drivers da base e braço: `KEEP_LAST(1)`, `BEST_EFFORT`; correto
  para estado/comando latest-value e para impedir backpressure no loop;
- LiDAR e IMU: `qos_profile_sensor_data` (`BEST_EFFORT`, histórico curto);
- imagem no detector: `BEST_EFFORT`, depth 1; saída AprilTag: reliable, depth 1;
- EKF: filas 2 para odom/IMU; evita backlog;
- teleop `/cmd_vel`: depth 1 e watchdog; adequado;
- QoS efetivo de `/joint_states`, `/wheel/odom`, `/odom` e endpoints Nav2 deve
  ser confirmado com `ros2 topic info -v` na Banana Pi antes de mudar defaults
  de pacotes externos.

Os drivers Python usam o executor simples padrão e um timer de I/O; não há
`MultiThreadedExecutor` ou callback groups adicionados. O I/O físico pesado fica
fora do executor do controller manager, atravessando pontes latest-value. O
driver SO-101 serializa acesso com um único lock; base executa leitura/escrita
no mesmo ciclo. Não há loop de runtime sem espera no caminho canônico.

Logging de falhas repetidas do braço já tem throttle de 2 s. Comando inválido
da base não está throttled, mas só é emitido por entrada inválida; se testes na
Banana mostrarem spam, aplicar throttle ali sem esconder a primeira falha.

## Segurança e caminho físico

```text
Nav2/teleop -> /cmd_vel -> mecanum controller
 -> velocity rad/s por roda -> MariolaSystem -> WheelCommand depth 1
 -> base_hardware_node -> escala/sinal/quantização -> brick + serial
 -> drivers de motor -> motores

encoders -> ticks -> drivers físicos -> posição rad + velocidade rad/s
 -> raw_joint_states -> MariolaSystem -> /joint_states + /wheel/odom
 -> EKF(+gyro Z) -> /odom + TF odom/base_footprint -> Nav2
```

Watchdogs em camadas:

- teleop para zero após 0,30 s sem joystick;
- `base_controller.reference_timeout=0.25 s`;
- driver da base zera após 0,30 s sem `WheelCommand`;
- plugin `MariolaSystem` publica zero após 0,50 s sem estado físico;
- três falhas consecutivas de I/O da base param o backend e encerram o driver;
- cinco falhas consecutivas de braço encerram o driver;
- encerramento de driver/control manager/RSP é fatal no launch.

Isso fornece fail-safe no SBC, mas o código auditado não prova um watchdog
independente no microcontrolador. Deve-se testar desligando fisicamente o DDS e
o processo Python; segurança real exige que o firmware também zere motores sem
heartbeat.

Unidades rastreadas: `/cmd_vel` em m/s e rad/s; juntas de roda em rad/rad/s;
encoders em ticks convertidos por 1644/3288 ticks/volta; braço em radianos na
interface ROS, com conversão LeRobot/Feetech na borda; garra converte a posição
do motor para a junta prismática em metros. Sinais/inversões são parâmetros por
roda. Esses valores não foram recalibrados sem o robô.

## F. Nova arquitetura

```text
bringup.launch.py                         ponto canônico/produção
├── hardware.launch.py                    CRÍTICO
│   ├── drivers físicos
│   ├── robot_state_publisher
│   ├── hardware_readiness
│   └── controller_manager + spawners
├── sensors.launch.py                     IMPORTANTE, selecionável
│   ├── LiDAR (off no perfil canônico)
│   └── IMU
├── localization.launch.py                IMPORTANTE
│   └── EKF
├── perception.launch.py                  OPCIONAL, off por padrão
│   └── câmera -> rectify -> AprilTag
└── manipulation.launch.py                IMPORTANTE
    └── controller_readiness -> move_group

workstation.launch.py                     fora do robô
└── RViz / teclado / joy / Xbox, opt-in

robot.launch.py                           alias compatível
```

Perfil padrão: base + braço + IMU + EKF + MoveIt. LiDAR fica desligado enquanto
não houver consumidor Nav2/SLAM integrado; visão fica desligada por CPU/USB.

```bash
ros2 launch bringup bringup.launch.py
ros2 launch bringup hardware.launch.py
ros2 launch bringup sensors.launch.py enable_lidar:=true enable_imu:=false
ros2 launch bringup bringup.launch.py enable_lidar:=true
ros2 launch bringup bringup.launch.py enable_perception:=true
```

## G/H. Implementação e impacto

| Antes | Depois | Motivo | Impacto |
|---|---|---|---|
| `robot.launch.py` era entrada implícita | `bringup.launch.py` canônico; alias preservado | ponto único claro sem quebrar scripts | manutenção e operação previsíveis |
| sensores forçados ao Python do sistema | env explícita, venv ativo/workspace, fallback | um mesmo ambiente contém ROS e drivers | elimina falhas divergentes de import |
| toda saída de sensor/percepção/EKF/MoveIt derrubava o robô | flags `*_failure_is_fatal`, false por padrão | criticidade depende da missão | periférico opcional não mata movimento |
| MoveIt iniciava junto dos spawners | consulta assíncrona `list_controllers` e só inicia com três controllers ativos | remove race sem sleep | startup determinístico |
| IMU 50 Hz, EKF 20 Hz | ambos 20 Hz; calibração 100→40 amostras preservando 2 s | não processar amostras descartáveis | menos serial/callbacks, mesma janela |
| LiDAR e percepção no perfil completo | opt-in | nenhum Nav2 integrado e visão é cara | menor CPU/RAM/USB em repouso |
| RSP sem política crítica explícita | saída do RSP encerra hardware | ausência de TF estrutural é insegura | falha visível e segura |
| ABI de interfaces antiga | rebuild limpo de `interfaces` | artefatos IDL removidos persistiam | imports e testes voltaram a carregar |

Não foi usado respawn. Drivers críticos não reiniciam em loop e não escondem
falhas de porta/firmware. Perfis incoerentes (`MoveIt` sem braço, sensores sem
base, percepção sem TF do braço) falham cedo com mensagem explícita.

## I. Checklist de validação progressiva

Antes de cada teste, use a imagem Python final e confirme dispositivos/permissões:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export CBR_SENSOR_PYTHON=~/ros2_ws/.venv/bin/python
python -c "import rclpy, serial, gpiod, pupil_apriltags, lerobot"
ros2 doctor --report
```

1. **Hardware isolado:** `hardware.launch.py enable_arm:=false`, verificar
   `/base_hardware/raw_joint_states`, zero após parar `/cmd_vel` e depois repetir
   com braço apenas. Acionar E-stop físico.
2. **Sensores:** iniciar IMU e LiDAR separadamente; medir `topic hz/bw`; cobrir
   sensor e desconectar cabo para conferir política não fatal/fatal.
3. **TF:** `ros2 run tf2_tools view_frames`; garantir um único
   `odom -> base_footprint`, nenhuma órfã e timestamps atuais.
4. **Odometria:** suspender rodas, validar sinais, ticks/volta e unidades;
   depois deslocar distância/ângulo medidos e comparar `/wheel/odom` e `/odom`.
5. **Localização:** testar EKF parado e em giro; só então AMCL/SLAM externo,
   verificando `map -> odom`.
6. **Controle:** enviar comandos limitados, medir timeout de zero e desconectar
   o publisher, DDS, driver e microcontrolador separadamente.
7. **Navegação:** após criar/validar o launch Nav2, conferir QoS de scan/odom,
   lifecycle ativo, collision monitor e 20 Hz efetivos.
8. **Sistema completo:** rodar 8–24 h por modo (idle, movimento, percepção),
   registrar crashes, deadlines, temperatura, CPU e RSS.

Comandos de evidência:

```bash
ros2 node list
ros2 topic list
ros2 topic info -v /cmd_vel
ros2 topic info -v /scan_front
ros2 topic hz /joint_states /wheel/odom /odom /imu/data
ros2 topic bw /camera/image_raw /scan_front
ros2 service list
ros2 action list
ros2 control list_controllers
ros2 lifecycle nodes
pidstat -p ALL 1
ps -eLo pid,tid,psr,pcpu,rss,comm --sort=-pcpu
cat /sys/class/thermal/thermal_zone0/temp
```

Critérios mínimos: nenhum comando após o watchdog; zero conflito TF; nenhum
backlog crescente; p95 de período dentro de ±20% em carga; sem OOM/crash;
temperatura abaixo do limiar de throttling da placa; nenhum processo opcional
derrubando os críticos.

## J. Métricas e validação obtida

| Métrica | Antes | Depois/estado validado |
|---|---:|---:|
| processos persistentes do perfil | 11 com visão+LiDAR | 7 no padrão; +1 LiDAR; +3 visão |
| processos transitórios | readiness + 4 spawners | 2 readiness + 4 spawners; todos encerram |
| IMU | 50 Hz | 20 Hz |
| controller/base/braço | 30 Hz | 30 Hz |
| detecção/câmera | 10/15 Hz sempre no perfil | 0 por padrão; 10/15 quando habilitada |
| LiDAR | contínuo | 0 por padrão; ~5 scans/s quando habilitado |
| CPU histórica com visão | 84–87% por núcleo, registro anterior do projeto | não medida sem Banana Pi |
| CPU histórica sem visão | 36–39% por núcleo, registro anterior do projeto | não medida sem Banana Pi |
| RAM/threads/temperatura | sem baseline confiável | medir pelo protocolo acima |

Validação de software efetuada:

- build limpo de 17 packages até `bringup`: sucesso;
- expansão dos launches principais com `--show-args`: sucesso;
- `check_urdf` do robô composto: sucesso;
- teste real de política: LiDAR e IMU falharam neste host sem hardware e o
  launch de sensores terminou sem shutdown global: sucesso;
- testes do bringup após a mudança: 26 passam; sobra uma divergência Nav2 de
  `cost_scaling_factor` 10 vs teste 8;
- conjunto por package: 192 testes passam e 6 falham por divergências já
  existentes (Nav2, janela LiDAR, safety do URDF, cubo, TCP e pre-grip).

Essas seis divergências são bloqueios de validação, não resultados desta
refatoração. Devem ser resolvidas confrontando cada teste com desenho mecânico,
calibração e ensaio físico, jamais escolhendo o valor que apenas deixa o teste
verde.
