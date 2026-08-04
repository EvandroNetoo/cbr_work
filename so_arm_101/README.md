# SO-ARM-101

Metapacote do módulo do braço SO-ARM-101.

Pacotes incluídos:

- `so_arm_101_description`: URDF/Xacro e meshes;
- `so_arm_101_bringup`: Gazebo, RViz, controladores e launchs;
- `so_arm_101_teleop`: controle pelo teclado;
- `so_arm_101_moveit_config`: planejamento e execução do braço com MoveIt 2.
- `so_arm_101_hardware`: ponte para o braço físico via LeRobot/Feetech.

## Compilar somente o módulo

Na raiz do workspace:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/cbr_work \
  --packages-select so_arm_101 so_arm_101_description \
  so_arm_101_bringup so_arm_101_teleop \
  so_arm_101_moveit_config so_arm_101_hardware
source install/setup.bash
```

## Iniciar a simulação

```bash
ros2 launch so_arm_101_bringup keyboard_control.launch.py
```

## Braço físico via LeRobot

Instale a variante Feetech do LeRobot no venv do workspace e calibre o follower
antes de iniciar:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
uv venv --python /usr/bin/python3 .venv
uv pip install --python .venv/bin/python -r requirements/lerobot-feetech.txt
source .venv/bin/activate
ros2 launch so_arm_101_hardware real.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

O hardware publica `/joint_states` e aceita os mesmos tópicos de trajetória do
braço (`/arm_controller/joint_trajectory` e
`/gripper_controller/joint_trajectory`). Consulte o README do pacote para
detalhes da calibração.

## Dependências do MoveIt 2

O OMPL gera o plano e o Simple Controller Manager entrega a trajetória ao
`arm_controller`. Instale ambos antes de usar o MoveIt:

```bash
sudo apt update
sudo apt install ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-simple-controller-manager
```

Também é possível instalar todas as dependências declaradas pelos pacotes:

```bash
cd /home/evandro/ros2_ws
rosdep install --from-paths src/cbr_work --ignore-src -r -y
```

## MoveIt 2

Para iniciar apenas o `move_group` (o robô e o hardware devem estar ativos):

```bash
ros2 launch so_arm_101_moveit_config move_group.launch.py
```

Para iniciar Gazebo, MoveIt e RViz juntos:

```bash
ros2 launch so_arm_101_moveit_config demo.launch.py
```

O `move_group` monitora `/joint_states` continuamente e conecta ao
`arm_controller` quando sua action fica disponível. Antes de executar a
primeira trajetória, aguarde o `arm_controller` aparecer como `active`.

Para conferir a comunicação durante um diagnóstico:

```bash
ros2 topic echo /joint_states --once
ros2 action info /arm_controller/follow_joint_trajectory
ros2 control list_controllers
```

Os grupos `arm` e `gripper` são executáveis pelos respectivos
`JointTrajectoryController`. A garra comanda apenas `right_clamp`;
`left_clamp` acompanha o movimento pela relação `mimic` do URDF.

## Ver os pacotes do módulo

```bash
ros2 pkg prefix so_arm_101
ros2 pkg prefix so_arm_101_description
ros2 pkg prefix so_arm_101_bringup
ros2 pkg prefix so_arm_101_teleop
ros2 pkg prefix so_arm_101_moveit_config
```
