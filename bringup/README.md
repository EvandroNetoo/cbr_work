# bringup

Ponto único de inicialização do robô físico e da workstation.

```text
robot.launch.py
├── hardware.launch.py       drivers, descrição, ros2_control, controllers
├── sensors.launch.py        LiDAR e IMU, sempre junto da base
├── localization.launch.py   EKF roda/IMU, sempre junto da base
├── perception.launch.py     câmera, rectify e AprilTag
├── manipulation.launch.py   move_group e servidor de manipulação
└── autonomy.launch.py       mapa, AMCL, Nav2 e gerenciador opcionais

workstation.launch.py        RViz, teclado e controle Xbox opcionais
```

O perfil padrão inicia tudo que está implementado para a competição:

```bash
ros2 launch bringup robot.launch.py
```

Argumentos de subsistema:

- `enable_base:=true`: base, LiDAR, IMU e EKF;
- `enable_arm:=true`: braço e garra;
- `enable_perception:=true`: câmera, retificação e AprilTag;
- `enable_moveit:=true`: `move_group` (requer braço).
- `enable_navigation:=true`: mapa, AMCL e Nav2 (requer base);
- `enable_mission:=true`: missão/manipulação (requer todos os subsistemas).

Drivers, sensores, percepção, descrição e subsistemas independentes começam em
paralelo. Uma única barreira orientada às mensagens físicas evita que os
controllers adotem zero antes da primeira leitura real; ela não espera IMU e
não usa polling de tópicos. Depois, os spawners aguardam o serviço nativo de
`/controller_manager` e ativam os controllers em paralelo. Não há `sleep` ou
`TimerAction`. Falha de driver, processo crítico ou ativação de controller
encerra o launch para que o supervisor externo possa reiniciá-lo.

Na workstation:

```bash
ros2 launch bringup workstation.launch.py
ros2 launch bringup workstation.launch.py enable_keyboard_teleop:=true
ros2 launch bringup workstation.launch.py enable_xbox_teleop:=true
```

No Xbox, `RB` é o botão obrigatório de habilitação, `LB` ativa turbo, o stick
esquerdo comanda X/Y e o stick direito horizontal comanda yaw. Ajuste eixos,
botões, deadzone e velocidades na seção `xbox_base_teleop` de
`config/controllers.yaml` se o driver Linux apresentar outro mapeamento.

O teleop normaliza a velocidade X/Y na diagonal, preservando sua direção. A
proteção final fica no `base_hardware_interface`: as quatro rodas são escaladas
juntas quando necessário, portanto comandos vindos de teleop, Nav2 ou scripts
não são rejeitados por excesso de velocidade.

O RViz usa `config/telemetry.rviz`, a única configuração principal, incluindo
RobotModel, LiDAR, odometria, TF, câmera e o painel MotionPlanning.

Nav2/AMCL são habilitados apenas no perfil autônomo. O SLAM permanece separado
para não publicar `map → odom` simultaneamente ao AMCL.
