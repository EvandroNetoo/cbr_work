# cbr_base_hardware

Driver Python exclusivo da base Mariola. Ele recebe as quatro velocidades em
`/cbr_base_hardware/command_velocities` e publica posição/velocidade dos
encoders em `/cbr_base_hardware/raw_joint_states`. Esses tópicos são internos;
o `/joint_states` público pertence ao `joint_state_broadcaster`.

Porta da expansão, IDs, inversões, escala de velocidade e ticks por volta ficam
todos em `config/hardware.yaml`. O brick permanece na SERIAL0, como no exemplo
validado. Com os valores atuais, `100 = 7 rad/s`; com raio de 0,034 m, isso
corresponde a 0,238 m/s na borda da roda.

As leituras de encoder permanecem a 30 Hz. Alvos que resultam no mesmo comando
físico `-100..100` não são reenviados em todo ciclo; um heartbeat de 5 Hz mantém
o controlador atualizado. Mudanças e transições para zero são imediatas.

O launch expõe rollback para esse comportamento:

```bash
ros2 launch cbr_base_hardware driver.launch.py \
  deduplicate_commands:=true command_heartbeat_hz:=5.0
```

Os argumentos sobrescrevem os defaults de `config/hardware.yaml`. Use
`deduplicate_commands:=false` para reproduzir uma escrita a cada ciclo. Execute
o primeiro teste físico com o
robô suspenso. Quando uma virtualenv está ativa, o launch executa o driver com
`$VIRTUAL_ENV/bin/python`, permitindo usar as bibliotecas instaladas nela sem
configuração adicional.
