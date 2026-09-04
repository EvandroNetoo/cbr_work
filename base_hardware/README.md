# base_hardware

Driver Python exclusivo da base Mariola. Ele recebe as quatro velocidades em
`/base_hardware/command_velocities` e publica posição/velocidade dos
encoders em `/base_hardware/raw_joint_states`. Esses tópicos são internos;
o `/joint_states` público pertence ao `joint_state_broadcaster`.

Porta da expansão, IDs, inversões, escala de velocidade e ticks por volta ficam
todos em `config/hardware.yaml`. O brick permanece na SERIAL0, como no exemplo
validado. Com os valores atuais, `100 = 11 rad/s`; com raio de 0,034 m, isso
corresponde a aproximadamente 0,37 m/s na borda da roda.

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
