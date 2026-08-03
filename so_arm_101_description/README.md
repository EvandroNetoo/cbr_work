# SO-ARM-101 no ROS 2

## Uma única interface de controle

O teclado e aplicações externas usam a mesma interface no Gazebo e no braço
real:

- `/arm_controller/joint_trajectory`: trajetória das cinco juntas do braço;
- `/gripper_controller/commands` (`std_msgs/msg/Float64MultiArray`): contém
  exatamente uma posição, de `0.0 m` aberta a `0.037 m` fechada;
- `/joint_states`: contém a posição medida da junta `right_clamp`.

Não existe adaptador nem segundo comando para a garra. O controlador comanda
somente `right_clamp`; no Gazebo, `left_clamp` é uma junta passiva acoplada por
`mimic`, representando a engrenagem real.

## Simulação

```bash
source install/setup.bash
ros2 launch so_arm_101_description keyboard_control.launch.py
```

Teclas: `q/a`, `w/s`, `e/d`, `r/f`, `t/g` controlam o braço; uma pressão em
`y` envia a garra ao fim de curso fechado (`0.037 m`) e uma pressão em `h` a
envia ao fim de curso aberto (`0.0 m`).

## Hardware

Primeiro, o hardware `ros2_control` do braço real deve estar ativo e oferecer
os mesmos controladores e nomes de juntas usados na simulação. Depois:

```bash
ros2 launch so_arm_101_description teleop.launch.py
```

O plugin de hardware deve expor `right_clamp` na coordenada comum de abertura
da garra (`0.0..0.037 m`) e converter essa coordenada internamente para a
posição do servo. Isso mantém teclado, aplicação, tópicos e controladores
idênticos, sem introduzir um nó adaptador no grafo ROS.

Para comandar a garra sem teclado:

```bash
ros2 topic pub --once /gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0185]}"
```
