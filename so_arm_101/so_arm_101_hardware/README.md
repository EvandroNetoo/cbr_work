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
colcon build --symlink-install --base-paths src/work
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
Os argumentos `port`, `robot_id` e `calibration_file` só sobrescrevem o YAML
quando são informados explicitamente na linha de comando.

O torque inicia habilitado. Para soltar todas as juntas a qualquer momento,
sustente o braço e chame o serviço:

```bash
ros2 service call /so101_hardware_node/set_torque std_srvs/srv/SetBool \
  "{data: false}"
```

O driver continua publicando as posições medidas e ignora comandos enquanto o
torque está desligado. Para reativá-lo sem reiniciar o launch:

```bash
ros2 service call /so101_hardware_node/set_torque std_srvs/srv/SetBool \
  "{data: true}"
```

Antes de energizar os servos, o driver define a pose física atual como alvo
para evitar um salto para um comando antigo.

Os arquivos `config/so101_follower.json` e
`config/gripper_calibration.yaml` são calibrações de referência. Confirme IDs,
offsets, faixas e endpoints no hardware antes de comandar. A calibração manual,
quando necessária, é solicitada por:

```bash
ros2 service call /so101_hardware_node/calibrate std_srvs/srv/Trigger '{}'
```

O nó Python não publica actions nem recebe trajetórias. O plugin
`so_arm_101_hardware_interface/SO101System` faz a ponte para o
`ros2_control`; os controllers ficam entre o MoveIt e este driver.

## Ciclo de I/O e reconexão

No perfil padrão, a callback ROS apenas mantém o alvo mais recente. Um timer de
escrita amostrada a 60 Hz executa `sync_write` somente quando o alvo muda. A
leitura usa 30 Hz durante movimento e cai para 2 Hz após um segundo sem mudança de alvo e
com velocidades baixas. Um novo alvo restaura imediatamente a leitura a 30 Hz.
Leitura e escrita compartilham o mesmo lock, portanto nunca acessam a serial ao
mesmo tempo. Quando não há setpoint pendente, o timer de escrita fica cancelado
e a interface `ros2_control` também deixa de publicar comandos idênticos. Um
novo setpoint reativa o timer; ele permanece ativo por dois períodos após a
última mudança para não perder amostras de uma trajetória a 30 Hz. Uma reconexão
do driver força uma nova publicação.

Falhas de leitura ou escrita fecham à força o descritor serial anterior e tentam
recriar o follower a cada `reconnect_interval_sec`. Uma reconexão validada zera
o estado de falha e agenda o reenvio do alvo mais recente. O processo encerra se
não restabelecer a comunicação dentro de `reconnect_timeout_sec`, evitando que o
restante do sistema opere indefinidamente com estado congelado.
