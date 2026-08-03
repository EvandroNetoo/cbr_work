# SO-ARM-101

Metapacote do módulo do braço SO-ARM-101.

Pacotes incluídos:

- `so_arm_101_description`: URDF/Xacro e meshes;
- `so_arm_101_bringup`: Gazebo, RViz, controladores e launchs;
- `so_arm_101_teleop`: controle pelo teclado;
- `so_arm_101_moveit_config`: planejamento e execução do braço com MoveIt 2.

## Compilar somente o módulo

Na raiz do workspace:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/cbr_work \
  --packages-select so_arm_101 so_arm_101_description \
  so_arm_101_bringup so_arm_101_teleop \
  so_arm_101_moveit_config
source install/setup.bash
```

## Iniciar a simulação

```bash
ros2 launch so_arm_101_bringup keyboard_control.launch.py
```

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

Nesse launch, MoveIt e RViz só iniciam depois que o Gazebo publicar todas as
juntas do braço em `/joint_states` e a action
`/arm_controller/follow_joint_trajectory` estiver disponível. Assim, a pose
inicial do RViz vem do Gazebo e o botão **Execute** não disputa a inicialização
dos controladores.

Para conferir a comunicação durante um diagnóstico:

```bash
ros2 topic echo /joint_states --once
ros2 action info /arm_controller/follow_joint_trajectory
ros2 control list_controllers
```

O grupo `arm` é executável pelo `arm_controller`. O grupo `gripper` está
descrito para planejamento, mas o controlador atual da garra recebe
`Float64MultiArray`, não a ação `FollowJointTrajectory` exigida pelo MoveIt.

## Ver os pacotes do módulo

```bash
ros2 pkg prefix so_arm_101
ros2 pkg prefix so_arm_101_description
ros2 pkg prefix so_arm_101_bringup
ros2 pkg prefix so_arm_101_teleop
ros2 pkg prefix so_arm_101_moveit_config
```
