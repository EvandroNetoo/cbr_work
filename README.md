# CBR ROS 2 Workspace

Pacotes ROS 2 do robô da CBR. A Banana Pi executa o sistema autônomo; o
notebook é uma estação opcional de visualização, teleoperação e diagnóstico.

## Arquitetura

```text
Banana Pi: driver → ros2_control → controllers → MoveIt → autonomia
Notebook:  RViz / MotionPlanning / teleop / diagnóstico
```

O notebook não é necessário para o controle ou planejamento do robô.

## Pacotes principais

- `cbr_bringup`: perfil embarcado completo do robô.
- `cbr_lidar`: aquisição do LiDAR XV-11 e publicação de `/scan_front`.
- `cbr_imu`: aquisição da IMU a 50 Hz e fusão leve com a odometria das rodas.
- `cbr_camera`: aquisição e retificação da câmera, independentes do robô.
- `so_arm_101`: descrição, hardware, controllers, teleop e MoveIt do braço.
- `cbr_apriltag`: detector AprilTag, usando tópicos de câmera externos.

## Build

Na raiz do workspace (`~/ros2_ws`):

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

## Banana Pi

O perfil embarcado inicia hardware, `ros2_control`, controllers e `move_group`,
sem RViz, Gazebo ou teleop:

```bash
ros2 launch cbr_bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  robot_id:=so101_follower
```

Esse único launch também inicia `/dev/video1` em 320 x 240, carrega a
calibração intrínseca, publica `/camera/image_rect` e executa o detector dos
AprilTags `tag36h11` de IDs 0 a 14. O LiDAR XV-11 é ligado pelo relé e publica
o setor frontal em `/scan_front`, no frame `lidar_front_link`.

O sistema aguarda um estado completo das seis juntas antes de iniciar os
controllers. A IMU calibra o offset angular com o robô parado, publica
`/imu/data`, e o EKF combina seu giro Z com `/wheel/odom` para manter a saída
pública `/odom`. Falha inicial de conexão ou cinco falhas consecutivas de
comunicação encerram o processo para reinício por um supervisor externo,
como `systemd`.

## Notebook

Configure o mesmo domínio DDS da Banana Pi:

```bash
export ROS_DOMAIN_ID=10
```

Visualização do robô físico:

```bash
ros2 launch cbr_bringup telemetry.launch.py
```

Interface MotionPlanning do MoveIt:

```bash
ros2 launch so_arm_101_moveit_config moveit_rviz.launch.py
```

Teleoperação manual:

```bash
ros2 run so_arm_101_teleop keyboard_teleop
```

O `telemetry.launch.py` não inicia outro `robot_state_publisher` nem publica
`/joint_states`; ele consome os dados fornecidos pela Banana Pi.

## Simulação e modelo offline

```bash
ros2 launch so_arm_101_bringup sim.launch.py
ros2 launch so_arm_101_bringup sim.launch.py headless:=true
ros2 launch so_arm_101_bringup model_demo.launch.py
```

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
```

Os tópicos internos do hardware são `/so101_hardware/raw_joint_states` e
`/so101_hardware/command_positions`. A interface pública é `/joint_states` e
as actions dos controllers.

## Testes

```bash
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_description/test -v
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_bringup/test -v
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_hardware/test -v
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_moveit_config/test -v
```

O driver real exige LeRobot/Feetech e uma porta serial conectada; os testes
locais não movimentam automaticamente o braço.
