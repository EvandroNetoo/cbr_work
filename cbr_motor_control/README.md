# cbr_motor_control

Pacote ROS 2 em Python para controlar por velocidade as quatro rodas
mecanum/omnidirecionais do CBR. A implementação acompanha
`MariolaZero/exemplos/10-expansaoMotor/main.py` e `controleMotores.py`.

## Arquitetura

Há dois barramentos independentes:

1. Os motores traseiros pertencem ao brick em `SERIAL0`. A classe `Motores`
   envia as duas velocidades no mesmo pacote usando `ENVIA_MOTORES`.
2. Os motores dianteiros são placas de IDs `0` e `7` no barramento `SERIAL3`.
   Cada uma recebe `velocidade_motor`, que usa o modo PID da placa.
3. `ControleMotores` executa os dois barramentos em paralelo. Os motores
   dianteiros permanecem sequenciais porque compartilham a mesma serial.
4. As inversões da montagem são dianteiro esquerdo e traseiro direito.

Cada mensagem ROS gera um envio. Se as mensagens deixarem de chegar, o
watchdog manda velocidade zero para todos os motores após 0,5 s por padrão.

## Interface ROS 2

No modo padrão, o nó assina `/cmd_vel` (`geometry_msgs/msg/Twist`):

```text
/cmd_vel -> cinemática mecanum X -> quatro velocidades lógicas
         -> ControleMotores -> SERIAL0 + SERIAL3 -> placas
```

Convenção ROS:

- `linear.x > 0`: frente;
- `linear.y > 0`: deslocamento para a esquerda;
- `angular.z > 0`: giro anti-horário.

O tópico `/motor_speeds_applied` publica `Int16MultiArray` na ordem:

```text
[dianteiro_esquerdo, dianteiro_direito,
 traseiro_esquerdo, traseiro_direito]
```

Os valores são velocidades lógicas antes das inversões da montagem.

## Build

A dependência `pyserial` está na venv do workspace. Ative-a e use seu Python
no build:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
.venv/bin/python -m colcon build \
  --symlink-install \
  --base-paths src/cbr_work/cbr_motor_control
source install/setup.bash
```

## Execução

O hardware real é o padrão. Antes de iniciar, suspenda as rodas e deixe uma
forma física de cortar a alimentação disponível:

```bash
ros2 launch cbr_motor_control motor_control.launch.py
```

Para testar sem abrir as seriais:

```bash
ros2 launch cbr_motor_control motor_control.launch.py dry_run:=true
```

Envie um comando de avanço:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Observe as velocidades calculadas:

```bash
ros2 topic echo /motor_speeds_applied
```

Interrompa o publicador com `Ctrl+C`. O watchdog deve publicar quatro zeros.

## Comando direto das quatro velocidades

O modo direto ajuda a validar identificação e sentido de cada roda:

```bash
ros2 launch cbr_motor_control motor_control.launch.py \
  command_mode:=motor_speeds \
  max_motor_speed:=15.0

ros2 topic pub -r 10 /motor_speeds std_msgs/msg/Int16MultiArray \
  "{data: [10, 0, 0, 0]}"
```

Repita com um único valor `10` em cada posição. Se uma roda girar ao
contrário, altere apenas seu parâmetro `*_inverted` em `config/motors.yaml`.

## Parâmetros principais

Os valores de `config/motors.yaml` são usados por padrão. Os argumentos do
launch abaixo somente os sobrescrevem quando forem passados explicitamente na
linha de comando.

- `dry_run`: não abre serial quando `true`;
- `command_mode`: `cmd_vel` ou `motor_speeds`;
- `watchdog_timeout`: tempo máximo sem comandos;
- `max_motor_speed`: limite aplicado aos valores enviados aos motores;
- `max_linear_speed`: valor de `linear.x/y` que corresponde ao limite motor;
- `max_angular_speed`: valor de `angular.z` que corresponde ao limite motor;
- `expansion_port`: conector lógico Mariola (`3` no exemplo);
- `configure_on_start`: executa reset, freio, ganhos PID e calibração;
- `front_*_id` e `*_inverted`: IDs e orientação da montagem.

As velocidades dos motores usam a escala inteira aceita pelos controladores
(`-100` a `100`). `max_linear_speed` e `max_angular_speed` descrevem o comando
do corpo do robô e devem ser calibrados experimentalmente. O pacote ainda não
publica odometria nem integra `ros2_control`.
