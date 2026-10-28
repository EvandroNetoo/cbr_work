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

O perfil `config/face.yaml` usa `show_menu_when_idle: true`: o nó também
assina `/mission/active` (`std_msgs/msg/Bool`, reliable/transient_local).
Enquanto não há missão, libera o OLED para o menu dos botões. Durante a
missão, reserva `/tmp/cbr_mission_face.lock` e anima normalmente. Ao encerrar
ou perder o publicador de atividade, libera a tela. Para exibir a carinha
continuamente, configure `show_menu_when_idle: false`; nesse modo o menu
permanece pausado enquanto a carinha reserva o OLED.

| `active_operation` | Expressão |
| --- | --- |
| `finish` | `comemorar`, com fogos de artifício |
| Demais operações de `config/face.yaml` | Rosto próprio com o mesmo nome da operação |
| vazio ou operação desconhecida | `neutro` |
| `state_known: false`, em qualquer operação | `triste` |

Prévia dos desenhos na resolução do OLED: [animação](docs/faces_preview.gif)
e [quadro de referência](docs/faces_preview.png). As sequências de
`navigate`, `follow_wall` e `place_on_table` aparecem também neste
[quadro de movimentos](docs/faces_movement.png). A navegação não desenha setas.
As novas cenas de entrega, empilhamento, organização e fogos estão no
[quadro de cenas](docs/faces_scenes.png). Os quadros de recuperação,
armazenamento e língua estão no [quadro de transferências](docs/faces_transfer.png).
As cenas de entrega na mesa e coleta estão neste [quadro de coleta e entrega](docs/faces_table_pick.png).
A entrada, manipulação e saída da garra aparecem no [quadro da garra](docs/faces_gripper.png).

Para regenerar essas prévias sem conectar o OLED:

```bash
python3 src/cbr_work/base_hardware/tools/preview_faces.py
```

O rosto acompanha a operação informada pelo gerenciador, inclusive enquanto
aguarda a conclusão dessa operação. O tópico atual contém estado da carga e
operação ativa; não informa a velocidade real da base nem distingue sucesso
de cancelamento quando a operação fica vazia ao encerrar. Nessas condições o
rosto volta a `neutro` no modo contínuo. Perder o publicador também volta a
`neutro` nesse modo; o simples silêncio durante uma operação longa mantém
a expressão. No modo com menu, o fim da missão devolve a tela ao menu.

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
        navigate: navigate
        pick: pick
        store: store
        place_on_table: place_on_table
```

Valores disponíveis: os 13 nomes de operação da tabela de configuração,
`piscar`, `piscadinha` e `comemorar`. Os nomes antigos `neutro`, `feliz`,
`triste`, `bravo`, `surpreso`, `dormindo`, `piscando` e `correndo` continuam
aceitos como aliases dos novos desenhos. Todos os rostos mantêm animação,
com piscadas e balanço suave. Os rostos comuns olham para frente na maior
parte do ciclo de 14 segundos, com olhadas laterais curtas.
Os movimentos específicos são:

| Expressão/operação | Olhar e animação |
| --- | --- |
| `analyze_scene` | Para baixo, varrendo os lados continuamente a cada 1,8 s |
| `pick` | Garra desce até o cubo na mesa e o leva pelo alto; pupilas seguem a garra, mantendo o rosto bravo e a língua original |
| `place_on_table` | Garra traz o cubo de fora da tela até a mesa; pupilas seguem a garra até ela sair e depois o rosto comemora |
| `retrieve` | Cubo sobe pela direita, entrando por baixo e saindo pelo alto; pupilas acompanham |
| `store` | Mesmo rosto de `retrieve`, com cubo descendo pela direita, entrando por cima e saindo por baixo |
| `place_on_shelf` | Para cima, varrendo os lados a cada 2,4 s |
| `place_on_precision_table` | Olhos apertados, língua no desenho original e gota de suor |
| `place_in_container` | Garra entra com o cubo, solta acima da abertura e sobe enquanto ele cai; pupilas seguem o cubo |
| `stack` | Garra entra segurando o cubo de fora da tela e o apoia sobre o primeiro; pupilas seguem a garra |
| `organize_precision` | Sobrancelhas bravas e boca triste, mantendo os olhos semicerrados; rosto elevado, suor, sete slots e seis cubos trocando de lugar |
| `finish`, `comemorar` | Rosto de comemoração com foguetes, explosões e partículas no lugar das estrelas |

Em `retrieve` e `store`, o deslocamento do cubo por toda a altura dura 3,44 s,
começando e terminando completamente fora da tela. As línguas usam o desenho
original, acompanhando apenas o balanço do rosto.

Todos os cubos usam a velocidade média de referência do `stack`: aproximadamente
22,7 pixels por segundo. A duração é calculada pela distância percorrida.
No `stack`, o cubo móvel entra de fora da tela: percorre 49 pixels em 2,16 s.
No recipiente, a garra carrega o cubo por 1,81 s e para acima da borda;
depois de abrir, o cubo cai sozinho durante 0,79 s enquanto ela se recolhe.

As quatro cenas `pick`, `place_on_table`, `place_in_container` e `stack` têm
garra entrando e saindo sempre por cima. `stack` e `place_in_container` já
começam com a garra fechada segurando o cubo, ambos inteiramente acima da tela.
No recipiente, as pontas da garra nunca passam da abertura: o cubo é solto
acima dela e a queda acontece separadamente do recuo da garra.
Em `pick`, `stack` e `place_on_table`, as pupilas acompanham a garra durante
a aproximação, o transporte e a subida vazia. Em `place_in_container`,
as pupilas acompanham o cubo, inclusive na queda livre após a soltura. Na mesa, os olhos ficam abertos até a garra
sair e então retomam a expressão feliz de comemoração.

`store` e `retrieve` continuam sem garra, com o cubo atravessando a tela
em 3,44 s e as pupilas acompanhando o cubo. `organize_precision` também
continua sem garra: um cubo se move para a vaga adjacente a cada 0,83 s
(0,71 s em movimento), sem sobrepor os seis cubos.

Navegação e acompanhamento de parede têm sequências rápidas, com transições
suaves de 0,25 segundo entre as poses:

- `navigate`: padrão → esquerda (1) → padrão → direita (2), trocando a cada
  0,70 segundo; ciclo completo de 2,8 segundos e olhar lateral mais acentuado.
- `follow_wall`: apenas quadro 1 → quadro 2 → quadro 1, trocando a cada
  0,80 segundo: lateral para baixo no quadro 1 e lateral para cima no quadro 2,
  sem o rosto padrão nem a piscadinha do antigo quadro 3.
- `place_on_table`: descida de 2,69 s até a mesa, seguida de transição de 0,25 s
  para a expressão feliz original, com olhos em arco e brilhos. O cubo permanece
  na mesa enquanto a garra abre e se recolhe; a comemoração começa após sua saída.
- `pick`: o cubo começa sobre a mesa e aguarda a garra descer e fechar.
  Depois sobe com ela durante 2,69 s até sair completamente pelo alto.

Olhos fechados ou apertados mantêm o movimento da face; `piscar` e `piscadinha` reabrem os olhos
periodicamente. Suor e brilhos também se movem quando presentes.
Um nome inválido gera uma mensagem com a chave incorreta e as opções aceitas.
Operações omitidas mantêm os padrões. `unknown_state` tem prioridade sobre
os demais rostos.

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
