# bringup

O bringup oferece dois perfis distribuídos e mantém `robot.launch.py`
inalterado como rollback monolítico.

Na Banana Pi, `hardware.launch.py` inicia apenas drivers físicos e controle de
baixo nível: braço, base, LiDAR, IMU, VL53, um único `controller_manager`,
`joint_state_broadcaster` e os controllers. Na Raspberry Pi 4,
`processing.launch.py` inicia `robot_state_publisher`, EKF e, por padrão,
visão, localização/Nav2, MoveIt/manipulação e missão. RViz e teleop continuam
separados no perfil de workstation.

```bash
# Banana Pi
export ROS_DOMAIN_ID=10
ros2 launch bringup hardware.launch.py port:=/dev/ttyUSB0

# Raspberry Pi
export ROS_DOMAIN_ID=10
ros2 launch bringup processing.launch.py
```

O mapa padrão é `arena`. A troca usa somente o nome instalado:

```bash
ros2 launch bringup processing.launch.py map:=arena_nova3
```

Os quatro grupos opcionais são independentes e ficam ativos por padrão:

```bash
ros2 launch bringup processing.launch.py \
  enable_vision:=false \
  enable_navigation:=false \
  enable_manipulation:=false \
  enable_mission:=false
```

`enable_navigation` controla conjuntamente map server, AMCL e Nav2. O EKF e o
`robot_state_publisher` permanecem sempre ativos no perfil de processamento.
Quando `enable_manipulation` está ativo, MoveIt e o servidor semântico só são
iniciados depois que os controllers remotos do braço respondem como ativos.

O perfil distribuído usa câmera a 15 FPS, detector AprilTag limitado a 10 Hz e
`controller_manager` a 30 Hz. O braço amostra o setpoint mais recente a 60 Hz,
mas só escreve na serial quando ele muda; a taxa maior evita alias com o loop
de controle. A base mantém somente o comando mais recente e reenvia o alvo
estacionário a 5 Hz. O LiDAR permanece ativo no perfil de hardware; MoveIt
pode ser desligado com `enable_manipulation:=false`.

As poses cartesianas e as detecções usadas pela manipulação são expressas em
`arm_base_link`. No perfil composto, `base_link` pertence ao chassi e difere da
base do braço pelo yaw físico de 90 graus do suporte.

O servidor `manipulation_server` é iniciado depois da ativação dos controllers,
junto ao MoveIt. O gerenciador de missão pode ser desligado independentemente;
o notebook não é requisito para a autonomia.

No notebook, a workstation consome os tópicos publicados pelo robô sem iniciar
drivers, controllers ou outro `robot_state_publisher`:

```bash
ros2 launch bringup workstation.launch.py
ros2 launch bringup workstation.launch.py enable_keyboard_teleop:=true
ros2 launch bringup workstation.launch.py enable_xbox_teleop:=true
```

O RViz inclui modelo, LiDAR, odometria, câmera, MotionPlanning e painéis
preparados para Nav2. Para diagnóstico, o perfil leve de navegação também pode
ser iniciado isoladamente depois do map server/AMCL e da pose inicial:

```bash
ros2 launch bringup navigation.launch.py
```

O launch encontra o YAML e a árvore BT no `share/bringup`; nenhum caminho do
workspace fica gravado nos parâmetros. Para diagnóstico ainda é possível
sobrescrever `params_file`, mas isso não é necessário no uso normal. O launch
legado `telemetry.launch.py` continua disponível para abrir apenas o mesmo
painel RViz.

No Xbox, `RB` habilita o movimento, `LB` ativa turbo, o stick esquerdo comanda
X/Y, o stick direito comanda yaw e `B` publica parada e solicita cancelamento
dos goals Nav2. Soltar `RB` ou perder a conexão também publica parada.

Os drivers físicos podem levar cerca de 20 segundos para conectar na Banana
Pi. O launch aguarda até 45 segundos pelos estados completos das seis juntas
comandadas do braço, das quatro rodas e da IMU antes de iniciar os controllers.
A saída de qualquer driver encerra o perfil de hardware.

```bash
ros2 launch bringup hardware.launch.py \
  port:=/dev/ttyUSB0 \
  hardware_state_timeout:=60.0
```

Para liberar as juntas do braço para posicionamento manual a qualquer momento,
mantendo os estados publicados pelo robô composto:

```bash
ros2 service call /so101_hardware_node/set_torque std_srvs/srv/SetBool \
  "{data: false}"
```

Nesse modo os comandos do braço são ignorados. Sustente-o antes da chamada,
pois as juntas podem cair sob o próprio peso. Use `"{data: true}"` no mesmo
serviço para reativar o torque sem reiniciar o robô.

Para alterar as taxas de câmera e controle no perfil distribuído:

```bash
ros2 launch bringup hardware.launch.py \
  port:=/dev/ttyUSB0 \
  controller_update_rate:=50 \
  base_deduplicate_commands:=false

ros2 launch bringup processing.launch.py camera_framerate:=30.0
```

## Validação distribuída

Meça por 60 segundos em repouso e durante uso, ocultando threads no `htop`
(`H`) para comparar processos agregados em cada placa:

```bash
# Raspberry Pi
pidstat -durwt -p ALL 1 60
ros2 topic hz /camera/image_rect
ros2 topic hz /apriltags/detections_camera

# Banana Pi
pidstat -durwt -p ALL 1 60
ros2 topic hz /so101_hardware/raw_joint_states
ros2 topic hz /base_hardware/raw_joint_states
```

Os valores esperados são 14–16 Hz para a imagem, 9–11 Hz para o detector e
28–32 Hz para os dois estados de hardware. Compare os dois perfis sob a mesma
temperatura/frequência de CPU. A meta é reduzir pelo menos 25% da CPU agregada
com visão e 30% em repouso, sem falhas seriais ou regressão de trajetória/parada.

Os argumentos de hardware pertencem ao braço e ao gate geral. A base não possui
configuração por CLI: hardware, geometria e controllers ficam nos YAMLs dos
pacotes `base_hardware`, `base_bringup` e `bringup`.
