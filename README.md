# CBR ROS 2 Workspace

Pacotes ROS 2 do robô da CBR. A Banana Pi executa drivers e controle de baixo
nível; o Raspberry Pi 4 executa percepção, planejamento e autonomia. O notebook
é uma estação opcional de visualização, teleoperação e diagnóstico.

## Arquitetura

```text
Banana Pi:    drivers físicos → ros2_control → controllers
Raspberry Pi: câmera / EKF / AMCL / Nav2 / MoveIt / manipulação / missão
Notebook:     RViz / MotionPlanning / teleop / diagnóstico
```

Os dois computadores do robô devem usar o mesmo `ROS_DOMAIN_ID`. O controle de
baixo nível permanece operacional na Banana sem depender do laço de rede.

## Pacotes principais

- `bringup`: perfis distribuídos, perfil monolítico e navegação do robô.
- `lidar`: aquisição do LiDAR XV-11 e publicação de `/scan_front`.
- `imu`: aquisição da IMU e configuração da fusão com a odometria das rodas.
- `vl53_distance`: actions de posicionamento frontal e seguimento lateral de
  parede com dois VL53L0X.
- `camera`: aquisição e retificação da câmera, independentes do robô.
- `so_arm_101`: descrição, hardware, controllers, teleop e MoveIt do braço.
- `apriltag`: detector AprilTag, usando tópicos de câmera externos.
- `manipulation`: actions semânticas de coleta, carga e depósito.
- `mission_manager`: execução sequencial de missões sobre Nav2 e manipulação.
- `cbr_simulation`: arena e integração com Gazebo Sim; veja o [estado da simulação](cbr_simulation/README.md).

## Build

Na raiz do workspace (`~/ros2_ws`):

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

## Execução distribuída

Na Banana Pi, inicie apenas o hardware e o controle local:

```bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" banana
ros2 launch bringup hardware.launch.py \
  port:=/dev/ttyUSB0 \
  robot_id:=so101_follower
```

Na Raspberry Pi, inicie o processamento e a autonomia:

```bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" rasp
ros2 launch bringup processing.launch.py
```

Todos os componentes ficam habilitados por padrão. É possível retirar uma
lista do perfil completo ou iniciar somente uma lista positiva:

```bash
# Raspberry: tudo menos missão
ros2 launch bringup processing.launch.py disable_components:=mission

# Notebook: somente missão
ros2 launch bringup processing.launch.py components:=mission
```

Os nomes disponíveis são `rsp`, `ekf`, `camera`, `vision`, `localization`,
`navigation`, `moveit`, `manipulation` e `mission`. Separe vários nomes por
vírgulas. MoveIt e o servidor de manipulação podem ser selecionados
independentemente.

A localização usa `arena` por padrão. Para trocar o mapa, informe somente o
nome instalado, sem diretório e sem `.yaml`:

```bash
ros2 launch bringup processing.launch.py map:=arena_nova3
```

A câmera conectada ao Raspberry publica `/camera/image_rect`; o detector usa os
mesmos AprilTags e frames do perfil anterior. O LiDAR e a IMU continuam na
Banana, publicando `/scan_front` e `/imu/data` para AMCL/Nav2 e EKF no Raspberry.

O hardware aguarda estados completos do braço, base e IMU antes de ativar os
controllers. O componente `moveit` aguarda os controllers do braço antes de
iniciar; `manipulation` é iniciado separadamente. Falhas físicas continuam
encerrando o launch da Banana para reinício por um supervisor externo, como
`systemd`.

O perfil monolítico anterior permanece inalterado para rollback:

```bash
ros2 launch bringup robot.launch.py
```

## Notebook

Configure Cyclone DDS e o mesmo domínio das duas placas em cada terminal ROS:

```bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" notebook
```

Os perfis usam `eth0` nas placas, `wlan1` na Banana, `wlan0` no Raspberry e
`wlp0s20f3` no notebook Ubuntu original. No WSL, após reconstruir o pacote
`bringup`, use o perfil `wsl` para selecionar a interface virtual disponível:

```bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" wsl
ros2 launch bringup workstation.launch.py
```

Antes do primeiro uso, instale as dependências com
`rosdep` e reconstrua o pacote `bringup`. Ao trocar de middleware, pare os
processos anteriores e execute `ros2 daemon stop` antes de iniciar os novos.
No WSL espelhado, prefira `ros2 topic list --no-daemon --spin-time 5`; se o
comando sem essa opção expirar, execute `ros2 daemon start` novamente.
No WSL 2 com NAT, o RViz pode abrir sem descobrir as placas; veja
[rede, WSL, validação e retorno ao Fast DDS](bringup/README.md#rede-e-cyclone-dds).

Workstation com RViz e MotionPlanning:

```bash
ros2 launch bringup workstation.launch.py
```

O painel antigo de telemetria também continua disponível:

```bash
ros2 launch bringup telemetry.launch.py
```

Teleoperação manual:

```bash
ros2 run so_arm_101_teleop keyboard_teleop
ros2 launch bringup workstation.launch.py enable_xbox_teleop:=true
```

No Xbox, segure `RB` para mover, use `LB` para turbo e pressione `B` para
parar e solicitar o cancelamento dos goals Nav2. A desconexão do controle
também publica parada. Nav2 faz parte do perfil de processamento; SLAM continua
fora do bringup de produção.

## Simulação e modelo offline

O robô completo pode ser inspecionado offline, com controles gráficos para as
juntas, sem iniciar drivers ou acessar o hardware:

```bash
ros2 launch robot_description display.launch.py
```

Os launches de Gazebo e as demos offline continuam fora do escopo atual.

## Diagnóstico

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action info /arm_controller/follow_joint_trajectory
ros2 topic echo /tf --once
ros2 topic hz /imu/data
ros2 topic echo /wheel/odom --once
ros2 action info /vl53/follow_wall
```

Os tópicos internos do hardware são `/so101_hardware/raw_joint_states` e
`/so101_hardware/command_positions`. A interface pública é `/joint_states` e
as actions dos controllers.

## Testes

```bash
/usr/bin/python3 -m pytest src/work/so_arm_101/so_arm_101_description/test -v
/usr/bin/python3 -m pytest src/work/so_arm_101/so_arm_101_bringup/test -v
/usr/bin/python3 -m pytest src/work/so_arm_101/so_arm_101_hardware/test -v
/usr/bin/python3 -m pytest src/work/so_arm_101/so_arm_101_moveit_config/test -v
```

O driver real exige LeRobot/Feetech e uma porta serial conectada; os testes
locais não movimentam automaticamente o braço.
