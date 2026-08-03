# SO-ARM-101

Metapacote do módulo do braço SO-ARM-101.

Pacotes incluídos:

- `so_arm_101_description`: URDF/Xacro e meshes;
- `so_arm_101_bringup`: Gazebo, RViz, controladores e launchs;
- `so_arm_101_teleop`: controle pelo teclado.

## Compilar somente o módulo

Na raiz do workspace:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/cbr_work \
  --packages-select so_arm_101 so_arm_101_description \
  so_arm_101_bringup so_arm_101_teleop
source install/setup.bash
```

## Iniciar a simulação

```bash
ros2 launch so_arm_101_bringup keyboard_control.launch.py
```

## Ver os pacotes do módulo

```bash
ros2 pkg prefix so_arm_101
ros2 pkg prefix so_arm_101_description
ros2 pkg prefix so_arm_101_bringup
ros2 pkg prefix so_arm_101_teleop
```
