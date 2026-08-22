# bringup

`robot.launch.py` é o perfil embarcado do robô composto. Ele executa os
drivers do braço e da base, um único `controller_manager`, um único
`joint_state_broadcaster`, os controllers e o `move_group` na Banana Pi, sem
iniciar RViz, Gazebo ou teleop. Ele também inicia sempre o driver da câmera, a
retificação calibrada e o detector AprilTag. O notebook é usado apenas para
visualização.

O perfil otimizado usa câmera a 15 FPS, detector AprilTag limitado a 10 Hz e
`controller_manager` a 30 Hz. Braço e base mantêm somente o comando mais recente
e reenviam alvos estacionários a 5 Hz. LiDAR e MoveIt continuam sempre ativos
nesta fase.

A futura lógica autônoma deve ser incluída neste perfil depois que os
controllers e o `/move_action` estiverem prontos; o notebook não é requisito
para essa inicialização.

No notebook, o painel principal de telemetria consome os tópicos publicados
pelo robô sem iniciar drivers, controllers ou outro `robot_state_publisher`:

```bash
ros2 launch bringup telemetry.launch.py
```

O painel usa `odom` como frame fixo e deixa habilitados o modelo do robô,
`/scan_front` e `/odom`. A árvore TF e `/camera/image_rect` ficam configuradas,
mas desligadas por padrão para serem ativadas durante diagnósticos. A interface
MotionPlanning continua separada no pacote do MoveIt.

Os drivers físicos podem levar cerca de 20 segundos para conectar na Banana
Pi. O launch aguarda até 45 segundos pelos estados completos das seis juntas
comandadas do braço e das quatro rodas antes de iniciar os controllers. A saída
de qualquer driver encerra todo o stack.

```bash
ros2 launch bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  hardware_state_timeout:=60.0
```

Rollback completo para o comportamento anterior de aquisição/controle:

```bash
ros2 launch bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  camera_framerate:=30.0 \
  controller_update_rate:=50 \
  arm_buffer_commands:=false \
  arm_deduplicate_commands:=false \
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
28–32 Hz para os dois estados de hardware. Compare o perfil padrão e o rollback
sob a mesma temperatura/frequência de CPU. A meta é reduzir pelo menos 25% da
CPU agregada com visão e 30% em repouso, sem falhas seriais ou regressão de
trajetória/parada.

Os argumentos acima pertencem ao braço e ao gate geral. A base não possui
configuração por CLI: hardware, geometria e controllers ficam nos YAMLs dos
pacotes `base_hardware`, `base_bringup` e `bringup`.
