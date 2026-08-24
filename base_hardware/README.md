# base_hardware

Driver Python exclusivo da base Mariola. Ele recebe as quatro velocidades em
`/base_hardware/command_velocities` e publica posição/velocidade dos
encoders em `/base_hardware/raw_joint_states`. Esses tópicos são internos;
o `/joint_states` público pertence ao `joint_state_broadcaster`.

Porta da expansão, IDs, inversões, escala de velocidade e ticks por volta ficam
todos em `config/hardware.yaml`. O brick permanece na SERIAL0, como no exemplo
validado. Com os valores atuais, `100 = 7 rad/s`; com raio de 0,034 m, isso
corresponde a 0,238 m/s na borda da roda.

O limite independente calculado para a base é `vx = ±0,238 m/s`, `vy = ±0,238
m/s` e `vw = ±1,070 rad/s`. Esses limites não podem ser atingidos
simultaneamente: numa diagonal pura com `vx = vy`, cada componente fica em
`±0,119 m/s`; com rotação, a roda mais exigida determina o limite. O bridge
`base_hardware_interface` escala as quatro rodas proporcionalmente para manter
a direção do comando dentro de `7 rad/s`.

As leituras de encoder permanecem a 30 Hz. Alvos que resultam no mesmo comando
físico `-100..100` não são reenviados em todo ciclo; um heartbeat de 5 Hz mantém
o controlador atualizado. Mudanças e transições para zero são imediatas.

O launch expõe rollback para esse comportamento:

```bash
ros2 run base_hardware base_hardware_node --ros-args \
  --params-file $(ros2 pkg prefix base_hardware)/share/base_hardware/config/hardware.yaml
```

Os argumentos sobrescrevem os defaults de `config/hardware.yaml`. Use
`deduplicate_commands:=false` para reproduzir uma escrita a cada ciclo. Execute
o primeiro teste físico com o
robô suspenso. Quando uma virtualenv está ativa, o launch executa o driver com
`$VIRTUAL_ENV/bin/python`, permitindo usar as bibliotecas instaladas nela sem
configuração adicional.
