# bringup

Ponto único de inicialização do robô físico e da workstation.

```text
bringup.launch.py               ponto de entrada canônico
├── hardware.launch.py       drivers, descrição, ros2_control, controllers
├── sensors.launch.py        LiDAR e IMU, sempre junto da base
├── localization.launch.py   EKF roda/IMU, sempre junto da base
├── perception.launch.py     câmera, rectify e AprilTag
└── manipulation.launch.py   move_group no computador embarcado

robot.launch.py                 alias de compatibilidade
workstation.launch.py           RViz, teclado e controle Xbox opcionais
```

O perfil padrão inicia base, braço, IMU, EKF e MoveIt. LiDAR e percepção ficam
desligados: ainda não há Nav2/SLAM integrado para consumir o scan, e a visão é
o maior pico de CPU e depende de uma câmera USB opcional:

```bash
ros2 launch bringup bringup.launch.py
ros2 launch bringup bringup.launch.py enable_lidar:=true
ros2 launch bringup bringup.launch.py enable_perception:=true
```

Argumentos de subsistema:

- `enable_base:=true`: base, LiDAR, IMU e EKF;
- `enable_lidar:=true` e `enable_imu:=true`: sensores selecionáveis separadamente;
- `enable_arm:=true`: braço e garra;
- `enable_perception:=true`: câmera, retificação e AprilTag;
- `enable_moveit:=true`: `move_group` (requer braço).

Drivers, sensores, descrição e subsistemas independentes começam em paralelo.
Uma barreira orientada às mensagens físicas evita que os controllers adotem
zero antes da primeira leitura real. Depois, os spawners aguardam o serviço
nativo de `/controller_manager`; MoveIt só começa quando seus três controllers
estão realmente `active`. Não há `sleep` ou `TimerAction`.

Drivers de movimento, `controller_manager`, `robot_state_publisher` e falha de
ativação de controller encerram o launch. LiDAR, IMU, EKF, câmera, retificação,
AprilTag e MoveIt são importantes/opcionais e não derrubam o movimento por
padrão. Para uma missão em que algum deles seja obrigatório, use
`sensor_failures_are_fatal:=true`, `localization_failure_is_fatal:=true`,
`perception_failure_is_fatal:=true` ou `moveit_failure_is_fatal:=true`.

Os sensores usam o Python indicado por `CBR_SENSOR_PYTHON`; na ausência dele,
usam o venv ativo ou `.venv/bin/python`. A imagem de produção deve conter
`pyserial` e `gpiod` nesse mesmo ambiente. Não dependa de um terminal interativo
para selecionar o interpretador.

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

Não há Nav2/SLAM funcional no código atual; nenhum launch de mapping foi criado.
