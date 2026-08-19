# cbr_base_hardware

Driver Python exclusivo da base Mariola. Ele recebe as quatro velocidades em
`/cbr_base_hardware/command_velocities` e publica posição/velocidade dos
encoders em `/cbr_base_hardware/raw_joint_states`. Esses tópicos são internos;
o `/joint_states` público pertence ao `joint_state_broadcaster`.

Porta da expansão, IDs, inversões, escala de velocidade e ticks por volta ficam
todos em `config/hardware.yaml`. O brick permanece na SERIAL0, como no exemplo
validado. Com os valores atuais, `100 = 7 rad/s`; com raio de 0,034 m, isso
corresponde a 0,238 m/s na borda da roda.

O launch não possui argumentos:

```bash
ros2 launch cbr_base_hardware driver.launch.py
```

Ele usa somente `config/hardware.yaml`. Execute o primeiro teste físico com o
robô suspenso. Quando uma virtualenv está ativa, o launch executa o driver com
`$VIRTUAL_ENV/bin/python`, permitindo usar as bibliotecas instaladas nela sem
configuração adicional.
