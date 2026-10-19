# base_hardware

Driver Python exclusivo da base Mariola. Ele recebe as quatro velocidades em
`/base_hardware/command_velocities` e publica posição/velocidade dos
encoders em `/base_hardware/raw_joint_states`. Esses tópicos são internos;
o `/joint_states` público pertence ao `joint_state_broadcaster`.

Porta da expansão, IDs, inversões, escala de velocidade e ticks por volta ficam
todos em `config/hardware.yaml`. O brick permanece na SERIAL0, como no exemplo
validado. Com os valores atuais, `100 = 11 rad/s`; com raio de 0,034 m, isso
corresponde a aproximadamente 0,37 m/s na borda da roda.

Ao iniciar, o driver lê `obtem_calibracao_motores()` do brick e usa o
primeiro valor para a placa dianteira esquerda e o segundo para a direita.
Cada `calibracao_manual` recebe `(abs(valor) * 2, -abs(valor) * 2)` com o
multiplicador definido em `hardware.calibration_multiplier`. Os PID do brick e
da expansão e os ajustes de freio também estão em `config/hardware.yaml`.
Uma leitura inválida ou uma placa que não confirme a configuração impede a
partida do driver.

O bridge limita proporcionalmente as quatro rodas a `11 rad/s`, preservando a
direção do comando mecanum. Zero permanece uma parada inequívoca; qualquer
alvo não nulo recebe pelo menos magnitude `4` na escala física para compensar
a zona morta dos motores.

As leituras de encoder permanecem a 30 Hz. Alvos que resultam no mesmo comando
físico `-100..100` não são reenviados em todo ciclo; um heartbeat de 5 Hz mantém
o controlador atualizado. Mudanças e transições para zero são imediatas.

O launch expõe rollback para esse comportamento:

```bash
ros2 launch base_hardware driver.launch.py \
  deduplicate_commands:=true command_heartbeat_hz:=5.0
```

Os argumentos sobrescrevem os defaults de `config/hardware.yaml`. Use
`deduplicate_commands:=false` para reproduzir uma escrita a cada ciclo. Execute
o primeiro teste físico com o
robô suspenso. Quando uma virtualenv está ativa, o launch executa o driver com
`$VIRTUAL_ENV/bin/python`, permitindo usar as bibliotecas instaladas nela sem
configuração adicional.

## Rosto animado no OLED

O `driver.launch.py` também inicia `mission_face_node`, em processo separado do
controle dos motores. Os perfis `bringup hardware.launch.py`,
`bringup robot.launch.py` e `base_bringup real.launch.py` incluem esse launch,
portanto a tela inicia junto da base. As animações de `MariolaZero/libs/main.py`
estão embarcadas em `base_hardware/face_animation.py`; o deploy precisa somente
do pacote ROS, sem acesso à pasta `MariolaZero`.

Na Banana Pi, instale a biblioteca do display no Python usado pelo launch:

```bash
sudo apt-get install python3-luma.oled python3-pil
```

Para uma virtualenv sem acesso aos pacotes do sistema, instale `luma.oled`
nessa virtualenv com `python -m pip install luma.oled`.

O nó assina `/mission/state` (`interfaces/msg/ManipulationState`) com QoS
reliable/transient_local e profundidade 1, igual ao publicador da missão.
Assim, uma tela ligada depois do início da missão recebe o snapshot atual.
A animação roda a até 20 FPS, com transições suaves, mesmo quando nenhuma
nova mensagem é publicada.

| `active_operation` | Expressão |
| --- | --- |
| `navigate`, `finish`, `follow_wall` | `correndo`, com vento frontal |
| `pick`, `retrieve`, `stack`, `place_on_precision_table`, `organize_precision` | `bravo`, concentrado |
| `store` | `piscando` |
| `place_on_table`, `place_in_container`, `place_on_shelf` | `feliz` |
| `analyze_scene` | `surpreso` |
| vazio ou operação desconhecida | `neutro` |
| `state_known: false`, em qualquer operação | `triste` |

O rosto acompanha a operação informada pelo gerenciador, inclusive enquanto
aguarda a conclusão dessa operação. O tópico atual contém estado da carga e
operação ativa; não informa a velocidade real da base nem distingue sucesso
de cancelamento quando a operação fica vazia ao encerrar. Nessas condições o
rosto volta a `neutro`. Perder o publicador também volta a `neutro`; o simples
silêncio durante uma operação longa mantém a expressão.

Barramento, endereço, FPS, tópico e intervalo de reconexão ficam em
`config/face.yaml` (padrão: I2C 0, endereço `0x3C`). Se a tela estiver ausente
ou falhar, somente o nó do OLED tenta reconectar a cada 5 segundos.

As expressões também são configuradas no mesmo YAML. Basta trocar o valor
ao lado da operação; os comentários explicam cada situação:

```yaml
    expressions:
      idle: dormindo
      disconnected: neutro
      unknown_state: triste
      unknown_operation: neutro
      operations:
        navigate: correndo
        pick: bravo
        store: piscando
        place_on_table: feliz
```

Valores disponíveis: `neutro`, `feliz`, `triste`, `bravo`, `surpreso`,
`dormindo`, `piscando`, `correndo`. Um nome inválido gera uma mensagem com a
chave incorreta e as opções aceitas. Operações omitidas mantêm os padrões.
`unknown_state` tem prioridade sobre os demais rostos.

Após editar o arquivo, reinicie o launch para aplicar. Com `--symlink-install`,
o arquivo instalado aponta para o YAML de origem; em uma instalação sem
symlinks, recompile o pacote antes de reiniciar. A tabela acima mostra os
valores padrão; suas escolhas no YAML substituem esses valores.

Para testar apenas a tela, sem iniciar os motores:

```bash
colcon build --packages-select base_hardware --symlink-install
source install/setup.bash
ros2 run base_hardware mission_face_node --ros-args \
  --params-file "$(ros2 pkg prefix --share base_hardware)/config/face.yaml"
```

Em outro terminal com o mesmo ambiente ROS, simule uma operação:

```bash
ros2 topic pub --rate 1 --qos-durability transient_local \
  /mission/state interfaces/msg/ManipulationState \
  '{state_known: true, active_operation: navigate}'
```

Use esse publicador de teste com o gerenciador de missão desligado, para
manter uma única origem do estado. A saída do nó limpa o display.
Para iniciar somente o driver, sem o rosto:

```bash
ros2 launch base_hardware driver.launch.py enable_face:=false
```
