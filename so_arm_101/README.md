# SO-ARM-101

Os pacotes deste diretório fornecem descrição, ponte física, interface
`ros2_control`, controllers, teleop de teclado e configuração MoveIt do braço.
Os launchs físicos foram centralizados em `bringup`.

Braço, garra, percepção e MoveIt sem a base móvel:

```bash
ros2 launch bringup robot.launch.py enable_base:=false
```

Somente braço e garra, sem percepção ou MoveIt:

```bash
ros2 launch bringup robot.launch.py \
  enable_base:=false enable_perception:=false enable_moveit:=false
```

O teclado é uma ferramenta opcional da workstation:

```bash
ros2 launch bringup workstation.launch.py enable_keyboard_teleop:=true
```

`move_group` roda na Banana Pi. O RViz/MotionPlanning do notebook conecta-se
às actions e tópicos remotos; ele não inicia outro planejador.

Os tópicos internos são `/so101_hardware/raw_joint_states` e
`/so101_hardware/command_positions`. As interfaces públicas são
`/joint_states`, `/arm_controller/follow_joint_trajectory` e
`/gripper_controller/follow_joint_trajectory`.

Simulação não faz parte da arquitetura atual e seus launchs antigos foram
removidos. Os modelos e configurações permanecem disponíveis para trabalho
futuro, sem serem iniciados pelo perfil físico.
