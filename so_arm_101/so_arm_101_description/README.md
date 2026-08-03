# SO-ARM-101 no ROS 2

Este workspace é dividido em três pacotes:

- `so_arm_101_description`: URDF/Xacro e meshes do robô;
- `so_arm_101_bringup`: RViz, Gazebo, controladores e launch files;
- `so_arm_101_teleop`: nó de controle pelo teclado.

Não é necessário Docker.

## Compilar

Na raiz do workspace:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Em um novo terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/evandro/ros2_ws/install/setup.bash
```

## Verificar o URDF

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
xacro src/cbr_work/so_arm_101/so_arm_101_description/urdf/so_101.urdf.xacro \
  > /tmp/so101.urdf
check_urdf /tmp/so101.urdf
```

## Visualizar no RViz

```bash
source /home/evandro/ros2_ws/install/setup.bash
ros2 launch so_arm_101_bringup display.launch.py
```

Esse modo usa `joint_state_publisher_gui` para mover as juntas com sliders.

## Iniciar apenas a simulação

```bash
source /home/evandro/ros2_ws/install/setup.bash
ros2 launch so_arm_101_bringup sim.launch.py
```

Sem a janela gráfica do Gazebo:

```bash
ros2 launch so_arm_101_bringup sim.launch.py headless:=true
```

## Simulação completa com RViz e teclado

```bash
source /home/evandro/ros2_ws/install/setup.bash
ros2 launch so_arm_101_bringup keyboard_control.launch.py
```

Teclas:

```text
q/a  base
w/s  ombro
e/d  cotovelo
r/f  punho
t/g  rotação do punho
y/h  fechar/abrir a garra
Ctrl-C encerra
```

## Teclado com hardware real

O driver do braço real e seus controladores devem estar ativos primeiro. Eles
precisam oferecer os mesmos nomes de juntas e tópicos da simulação.

```bash
source /home/evandro/ros2_ws/install/setup.bash
ros2 launch so_arm_101_bringup teleop.launch.py
```

Esse launch não inicia Gazebo nem hardware. Ele inicia somente o teclado.

## Comandar a garra manualmente

`right_clamp` usa metros: `0.0` fechada e `0.037` aberta.

```bash
ros2 topic pub --once /gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0185]}"
```

## Comandar o braço manualmente

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [base_link_to_link1, link1_to_link2, link2_to_link3, link3_to_link4, link4_to_link5], points: [{positions: [0.2, -1.0, 1.5, 0.0, 0.0], time_from_start: {sec: 1}}]}"
```

## Diagnóstico rápido

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /joint_states
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 service list | sort
```

Os controladores esperados são:

```text
joint_state_broadcaster
arm_controller
gripper_controller
```

## Testes

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_description/test -v
python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_bringup/test -v
python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_teleop/test -v
```

O teste dos meshes requer o pacote Python `trimesh`.

## Gerar meshes de colisão

```bash
cd /home/evandro/ros2_ws/src/cbr_work/so_arm_101/so_arm_101_description
python3 scripts/generate_collision_meshes.py
```
