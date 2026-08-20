# so_arm_101_hardware

Driver ROS 2 de baixo nível para o SO-ARM-101 follower usando `SO101Follower`
e o barramento Feetech do LeRobot. O nó publica estados brutos em
`/so101_hardware/raw_joint_states` e recebe posições em
`/so101_hardware/command_positions`.

## Ambiente Python

O driver precisa acessar LeRobot e as bibliotecas ROS do Jazzy. Use um ambiente
Python compatível com essa instalação, sem instalar dependências no Python
global:

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
python3 -m venv .venv
source .venv/bin/activate
pip install 'lerobot[feetech]' PyYAML
python -c "import rclpy, lerobot, serial, scservo_sdk"
colcon build --symlink-install --base-paths src/cbr_work
source install/setup.bash
```

O launch procura automaticamente por `<workspace>/.venv/bin/python`. Portanto,
depois de criar a virtualenv, também é possível executar o comando sem ativá-la
manualmente. Para escolher outro interpretador explicitamente:

```bash
ros2 launch so_arm_101_hardware driver.launch.py \
  python_executable:=/caminho/para/.venv/bin/python \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

Como alternativa, defina `SO_ARM_101_PYTHON` no ambiente antes de iniciar o
launch.

## Driver isolado

Para diagnóstico, inicia somente o nó Python:

```bash
ros2 launch so_arm_101_hardware driver.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

O launch `real.launch.py` neste pacote é um wrapper compatível para o mesmo
driver. O sistema completo deve ser iniciado pelo pacote bringup:

```bash
ros2 launch so_arm_101_bringup real.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

Sem esses argumentos, `config/real.yaml` é a fonte dos parâmetros físicos.
Os argumentos `port`, `robot_id`, `calibration_file`, `buffer_commands`,
`deduplicate_commands` e `command_heartbeat_hz` só sobrescrevem o YAML quando
são informados explicitamente na linha de comando.

Os arquivos `config/so101_follower.json` e
`config/gripper_calibration.yaml` são calibrações de referência. Confirme IDs,
offsets, faixas e endpoints no hardware antes de comandar. A calibração manual,
quando necessária, é solicitada por:

```bash
ros2 service call /so101_hardware/calibrate std_srvs/srv/Trigger '{}'
```

O nó Python não publica actions nem recebe trajetórias. O plugin
`so_arm_101_hardware_interface/SO101System` faz a ponte para o
`ros2_control`; os controllers ficam entre o MoveIt e este driver.

## Ciclo de I/O e rollback

No perfil padrão, a callback ROS apenas mantém o alvo mais recente. Um único
ciclo a 30 Hz executa `sync_write` quando necessário e depois `sync_read`, sem
duas callbacks competindo pela serial. Durante movimento o alvo pode ser enviado
a 30 Hz; quando permanece idêntico, o heartbeat é de 5 Hz.

Para comparar com o caminho antigo sem recompilar:

```bash
ros2 launch so_arm_101_hardware driver.launch.py \
  port:=/dev/ttyUSB0 \
  buffer_commands:=false deduplicate_commands:=false
```

`buffer_commands:=true deduplicate_commands:=false` mantém a serial em um único
ciclo, mas escreve o último alvo em todos os ciclos de 30 Hz.
