# bringup

`robot.launch.py` é o perfil embarcado do robô composto. Ele executa os
drivers do braço e da base, um único `controller_manager`, um único
`joint_state_broadcaster`, os controllers e o `move_group` na Banana Pi, sem
iniciar RViz, Gazebo ou teleop. Ele também inicia sempre o driver da câmera, a
retificação calibrada e o detector AprilTag. O notebook é usado apenas para
visualização.

O perfil otimizado usa câmera a 15 FPS, detector AprilTag limitado a 10 Hz e
`controller_manager` a 30 Hz. O braço amostra o setpoint mais recente a 60 Hz,
mas só escreve na serial quando ele muda; a taxa maior evita alias com o loop
de controle. A base mantém somente o comando mais recente e reenvia o alvo
estacionário a 5 Hz. LiDAR e MoveIt continuam sempre ativos nesta fase.

As poses cartesianas e as detecções usadas pela manipulação são expressas em
`arm_base_link`. No perfil composto, `base_link` pertence ao chassi e difere da
base do braço pelo yaw físico de 90 graus do suporte.

A futura lógica autônoma deve ser incluída neste perfil depois que os
controllers e o `/move_action` estiverem prontos; o notebook não é requisito
para essa inicialização.

No notebook, a workstation consome os tópicos publicados pelo robô sem iniciar
drivers, controllers ou outro `robot_state_publisher`:

```bash
ros2 launch bringup workstation.launch.py
ros2 launch bringup workstation.launch.py enable_keyboard_teleop:=true
ros2 launch bringup workstation.launch.py enable_xbox_teleop:=true
```

O RViz inclui modelo, LiDAR, odometria, câmera, MotionPlanning e painéis
preparados para Nav2. Depois de iniciar o map server/AMCL e definir a pose
inicial, o perfil leve de navegação pode ser iniciado sem argumentos:

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
comandadas do braço e das quatro rodas antes de iniciar os controllers. A saída
de qualquer driver encerra todo o stack.

```bash
ros2 launch bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  hardware_state_timeout:=60.0
```

Para liberar as juntas do braço para posicionamento manual, mantendo os estados
publicados pelo robô composto:

```bash
ros2 launch bringup robot.launch.py \
  port:=/dev/ttyUSB0 disable_torque:=true
```

Nesse modo os comandos do braço são ignorados. Sustente-o antes da partida,
pois as juntas podem cair sob o próprio peso.

Para comparar o perfil atual com as taxas antigas de câmera/base:

```bash
ros2 launch bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  camera_framerate:=30.0 \
  controller_update_rate:=50 \
  base_deduplicate_commands:=false
```

## Validação na Banana Pi

Meça por 60 segundos em repouso e com AprilTag contínuo, ocultando threads no
`htop` (`H`) para comparar processos agregados:

```bash
pidstat -durwt -p ALL 1 60
ros2 topic hz /camera/image_rect
ros2 topic hz /apriltags/detections_camera
ros2 topic hz /so101_hardware/raw_joint_states
ros2 topic hz /base_hardware/raw_joint_states
```

Os valores esperados são 14–16 Hz para a imagem, 9–11 Hz para o detector e
28–32 Hz para os dois estados de hardware. Compare os dois perfis sob a mesma
temperatura/frequência de CPU. A meta é reduzir pelo menos 25% da CPU agregada
com visão e 30% em repouso, sem falhas seriais ou regressão de trajetória/parada.

Os argumentos acima pertencem ao braço e ao gate geral. A base não possui
configuração por CLI: hardware, geometria e controllers ficam nos YAMLs dos
pacotes `base_hardware`, `base_bringup` e `bringup`.
