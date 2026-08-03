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
`mimic`, representando a engrenagem real. Por isso `left_clamp` não deve ser
declarada como recurso de hardware no bloco `ros2_control`: isso criaria um
segundo laço de mimic que disputaria com a restrição física do Gazebo.

## Simulação

```bash
source install/setup.bash
ros2 launch so_arm_101_description keyboard_control.launch.py
```

Teclas: `q/a`, `w/s`, `e/d`, `r/f`, `t/g` controlam o braço; cada pressão em
`y`/`h` fecha/abre a garra gradualmente em `0.005 m`, até os limites
(`0.037 m` fechada e `0.0 m` aberta). O passo pode ser alterado pelo parâmetro
`gripper_step`. Na simulação, a velocidade da garra é limitada a `0.05 m/s`;
assim, o passo padrão de `0.005 m` leva aproximadamente `0.1 s` em vez de
acontecer em um único quadro.

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
