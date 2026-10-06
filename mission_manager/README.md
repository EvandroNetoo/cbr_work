# mission_manager

Executor por visitas das missões da RoboCup@Work. O pacote não controla motores
diretamente: ele compõe Nav2, alinhamento VL53 e as actions semânticas do pacote
`manipulation`.

## Arquivos

- `config/arena.yaml`: poses fixas, alturas, tipos, alinhamento, recuo e
  recuperação de coleta;
- `config/plans/*.yaml`: visitas e tarefas flexíveis selecionadas por `plan_id`;
- `config/mission_manager.yaml`: nomes das actions, serviço/tópico de estado,
  compartimentos disponíveis, proteções do `FollowWall` e timeouts ROS.

As poses de `arena.yaml` devem ser calibradas para a arena antes da execução. O nó inicia
normalmente, mas um goal retorna `CONFIGURATION_ERROR` sem movimentar o robô se
a arena ou o plano não forem válidos.

## Formato de missão v2

A ordem de `visits` define a rota. Dentro de cada visita, a ordem de `tasks`
não determina a execução: o executor considera identificação, carga e viabilidade
de concluir todas as visitas restantes. Tags não solicitadas são ignoradas.

```yaml
schema_version: 2
plan_id: coleta_flexivel
finish: true
visits:
  - id: coleta_ws1
    target: ws_1
    tasks:
      - {id: pegar_1, action: pick, tag_id: 1}
      - {id: pegar_2, action: pick, tag_id: 2}
      - {id: pegar_3, action: pick, tag_id: 3}
  - id: entrega_ws2
    target: ws_2
    tasks:
      - {id: entregar_1, action: place_on_table, tag_id: 1}
      - {id: entregar_2, action: place_in_container, tag_id: 2, container_color: red}
      - {id: pegar_4, action: pick, tag_id: 4}
  - id: entrega_ws3
    target: ws_3
    tasks:
      - {id: entregar_3, action: place_on_table, tag_id: 3}
      - {id: entregar_4, action: place_on_table, tag_id: 4}
```

IDs de visitas e tarefas são obrigatórios e únicos na missão. `tasks: []` é
permitido para visitas de navegação. `initial_location` tem padrão `start` e
informa a localização física inicial; cada visita ainda executa sua navegação.
`finish` tem padrão `false`; quando verdadeiro, navega ao ponto `finish` após
concluir as visitas. A arena permanece em `schema_version: 1`.

Cada coleta ou entrega informa `tag_id`. As entregas disponíveis são
`place_on_table`, `place_in_container` (`container_color: red|blue`),
`place_on_shelf` e `stack`. Uma pilha sem ordem fixa é declarada assim:

```yaml
- id: montar_pilha
  action: stack
  support_tag_id: 14
  tag_ids: [4, 5]
```

Qualquer membro pode ser o primeiro sobre 14. O próximo é colocado sobre o
último objeto depositado e confirmado. O suporte deve estar na visita e livre
para receber o objeto; não pode pertencer ao próprio grupo.

`store` e `retrieve` são automáticos e não são aceitos no YAML. A carga começa
vazia. O primeiro compartimento livre segue a ordem de `cargo_slot_ids`.
Na saída de uma visita, a garra fica vazia ou leva um objeto que será entregue
na próxima visita com tarefas. Visitas com `tasks: []` mantêm a navegação e
o alinhamento, mas permitem passar com esse objeto na garra, sem exigir
armazenamento adicional. No exemplo, a tag 3 viaja armazenada; não pode ser a última
coleta quando isso deixaria a garra bloqueada. A missão termina sem carga pendente.

Quando não há uma tarefa identificada disponível nem uma cena já analisada
na posição atual, o executor prepara a câmera com garra vazia e observa tags e
contêineres por dois segundos em `/vision/analyze_scene` (parâmetro `vision_action`). Prioriza entrega já na garra, identificação na posição atual,
memória com menor deslocamento lateral e IDs para desempate. Na ausência de
candidatos identificados, busca pelos pontos existentes e escolhe novamente a
cada observação. Não completa a busca de uma tag ausente antes de considerar
as outras. A memória conserva as posições das demais tags; só a tag coletada
é removida. Históricos são separados por área, posição e iluminação.

A detecção da análise explícita autoriza uma única coleta direta, imediatamente
após essa análise. O servidor `manipulation` valida o alcance pelo
`profiles.yaml` antes de planejar a pegada. Se a tag estiver alcançável, a coleta
usa essa detecção sem alinhar a base nem capturar novamente. Se estiver fora do
alcance, a recuperação alinha a base e solicita uma nova detecção.

Uma coleta, transferência, depósito, navegação ou reposicionamento consome essa
autorização. Voltar ao mesmo ponto não a renova. Sem essa autorização, uma tag
memorizada orienta o robô ao alinhamento configurado, e `PickObject` analisa
novamente a cena nessa posição antes de pegar. O ponto onde a tag foi vista
não é usado como destino de alinhamento.

O feedback mantém o contrato de `ExecuteMission`: `total_steps` inclui uma
navegação por visita, cada tarefa, cada objeto de uma pilha e o `finish` opcional.
Transferências internas não aumentam esse total. Seus erros apontam a tarefa
que motivou a transferência. Entregas mostram o destino efetivo no feedback;
`_delivery_outcomes` registra tarefa, tag, área, ação solicitada e efetiva, cor
do contêiner e suporte quando aplicáveis.

Planos v1 são rejeitados com orientação de migração. Os exemplos válidos foram
migrados pelos passos ativos, preservando visitas repetidas, tags e destinos.
`config/invalid_plans/` preserva os originais inconsistentes e os motivos; esses
arquivos não são instalados como planos executáveis. Em particular, o advanced
original empilha a tag 4 sobre si mesma, e `simples` não declara a origem da carga.

## Navegação

Para uma service area, `navigate` executa:

```text
PrepareManipulator(NAVIGATION) → NavigateToPose → FollowWall(travel=0)
```

Ao sair de uma service area para outro destino, o fluxo começa com:

```text
FollowWall(departure, travel=destino-atual) → PrepareManipulator(NAVIGATION) → NavigateToPose
```

Para `start` e `finish`, o alinhamento de chegada é omitido. Os blocos
`alignment` e `departure` de uma service area sobrescrevem parcialmente
`alignment_defaults` e `departure_defaults`, respectivamente.
`departure.lateral_position_mm` é uma coordenada absoluta relativa ao centro
registrado na chegada à mesa. Com o padrão `0`, o robô retorna a esse centro
enquanto se afasta da superfície no mesmo goal `FollowWall`.
`departure.max_alignment_error_mm` e
`departure.alignment_recovery_distance_mm` configuram a recuperação de
alinhamento desse retorno lateral. `departure.minimum_lateral_clearance_mm`
define a folga mínima para obstáculos no lado do movimento. Os três campos
podem ser sobrescritos em cada área; quando omitidos, são usados os parâmetros
globais `follow_wall.*`.
Se o recuo não precisar de deslocamento lateral, ambos são enviados como `0`,
pois a recuperação é uma manobra ao longo da parede.

Os limites `follow_wall.max_alignment_error_mm` e
`follow_wall.alignment_recovery_distance_mm` do `mission_manager.yaml` são
usados nos demais goals com deslocamento lateral.
`follow_wall.alignment_error_ignore_sec` define por quantos segundos, após a
primeira leitura VL53 válida, o limite de desalinhamento fica suspenso nesses
goals; o padrão é 2,0 s. Durante o alinhamento frontal de chegada, os três
campos são enviados como `0`; no recuo de uma mesa, os limites do bloco
`departure` são usados quando houver retorno lateral, com a mesma janela
configurada no mission manager. Aborto
durante o percurso lateral por desalinhamento, conclusão da
recuperação ou obstáculo na folga lateral mínima é registrado como aviso e o
fluxo da missão continua usando o deslocamento efetivamente medido. Quando a
folga lateral bloqueia um goal que também corrige a distância frontal, o
`FollowWall` mantém somente `linear.x` até estabilizar na distância solicitada
e então devolve o resultado parcial. Timeout, falha de sensores, odometria
inválida e comunicação continuam encerrando a missão.

## Áreas de serviço e prateleira (SH)

Câmera e LED são ativados na chegada a qualquer área `WS`, `SH` ou `PP`,
antes do alinhamento. Também são ativados se `initial_location` for uma área
de serviço. São desligados antes do recuo de saída e ao encerrar a missão,
inclusive em cancelamento ou falha.

Na SH, `pick` usa o mesmo fluxo AprilTag, busca lateral e recuperação da WS,
com o `height_cm` da área de coleta. O depósito alto usa explicitamente
`action: place_on_shelf` no plano: move o braço para a pose fixa configurada,
abre a garra e retorna à posição segura. O tipo `SH` não troca a ação do plano
automaticamente, e `height_cm` não define a altura desse depósito.

O plano `example_shelf` demonstra coleta e depósito na `sh_1`. Os valores de
`sh_1` em `config/arena.yaml` e as juntas de `place_on_shelf_high` no arquivo
`so_arm_101_moveit_config/config/so_arm_101.srdf` são **fictícios** e devem ser
substituídos pela calibração antes de executar no robô. O perfil `shelf` de
`manipulation/config/profiles.yaml` já aponta para essa pose e está habilitado.

## Recuo lateral antes do depósito

Cada resultado do `FollowWall` inclui a folga lateral do LiDAR para os lados
esquerdo e direito, medida desde o footprint. `has_fresh_lateral_scan` indica
se houve um scan recente; os campos `has_valid_*_lateral_clearance` indicam
se foi detectado obstáculo no respectivo lado. Sem obstáculo no alcance, o
campo desse lado fica inválido e o recuo não é necessário.

Antes de `StoreObject`, o mission manager compara a folga do LiDAR no lado
do compartimento (`left` ou `right`) com
`deposit_lateral_retreat.threshold_mm`, somente quando o último deslocamento
lateral foi na direção desse compartimento. Se a folga for menor, envia um
novo `FollowWall` no sentido oposto com percurso
`deposit_lateral_retreat.distance_mm`. O armazenamento só começa depois de
completar o recuo. Uma leitura obsoleta ou um limite de posição que impeça o
recuo bloqueia o armazenamento. Os valores são definidos em
`config/mission_manager.yaml`; configurar qualquer um como `0` desativa a
regra.
Essa verificação não se aplica a depósitos em mesa, contêiner ou prateleira,
nem ao empilhamento.

## Recuperação de coleta fora do alcance

No `stack`, o gerenciador pede uma detecção da tag de apoio antes de qualquer
soltura. Quando ela é encontrada, tenta alinhar a base com os alvos
`pickup_recovery.stack_preferred_tag_x_m` e `stack_preferred_tag_y_m` de
`config/arena.yaml`, independentemente dos alvos da coleta em SH. São posições
da tag no referencial do braço, em metros; os limites de movimento e as
tolerâncias continuam usando `pickup_recovery`. O alinhamento é tentado mesmo
com `pickup_recovery.enabled: false`, que apenas desativa a busca de tags.
Depois, o stack detecta novamente a tag e realiza o depósito.

Empilhamentos seguintes na mesma pilha reutilizam esse alinhamento quando a
base permanece na mesma posição, inclusive com `retrieve`/`store` entre eles.
O apoio pode ser o cubo recém-depositado ou uma tag já pertencente à pilha.
Navegação, reposicionamento da base, coleta, outro tipo de depósito, mudança
de pilha ou uma nova missão exigem um novo alinhamento. Uma tentativa limitada
pelas tolerâncias ou proteções segue a mesma regra do pick em SH: o alvo exato
não precisa ser alcançado para prosseguir. Uma falha de comunicação ou estado
físico incerto interrompe a operação.

Antes de qualquer depósito em uma área `SH`, o gerenciador recolhe o braço
com a carga e executa `FollowWall` para confirmar a distância frontal do VL53.
`shelf_place_alignment_defaults` define o padrão de 40 mm, tolerância de 5 mm
e timeout de 10 s. Cada SH pode sobrescrever apenas os campos desejados em
`service_areas.<id>.shelf_place_alignment`, por exemplo `distance_mm: 80`.
Essa distância é independente do alinhamento de chegada e da centralização
da AprilTag. O alinhamento preserva a posição lateral e uma falha impede
o envio da ação de depósito.

Áreas `SH` selecionam automaticamente `shelf_front`. Antes de pegar, uma
detecção solicita uma tentativa de alinhamento para
`pickup_recovery.shelf_preferred_tag_x_m/y_m`, mesmo com a tag já alcançável
e mesmo com `pickup_recovery.enabled: false`. Esse flag controla a recuperação
opcional e a busca, não a tentativa inicial da SH. A base respeita os
limites de parede e percurso existentes. Se já centralizada, não se move,
e reutiliza a detecção se a posição permanece a mesma. Depois de um
deslocamento, detecta novamente para atualizar o alvo. A posição alcançada é aceita mesmo fora das tolerâncias ou sem deslocamento. A nova
pose detectada segue ao MoveIt sem exigir centralização exata. WS e PP usam
`tabletop`.

Cada resultado de `PickObject`, bem-sucedido ou não, inclui todas as AprilTags
observadas enquanto a base permaneceu parada. O mission manager associa cada
detecção à distância atual da parede e a uma coordenada lateral, cuja origem é
o alinhamento de chegada à área. A memória é separada por área e permanece
válida durante toda a missão, inclusive depois de navegar para outra área.

Quando o filtro de alcance bloqueia a AprilTag, ou o MoveIt devolve o código
`99999` antes de fechar a garra, o resultado também inclui a pose específica
usada para recuperar a coleta. Se `pickup_recovery.enabled` estiver ativo, o
mission manager:

```text
PrepareManipulator(OBSERVATION) → FollowWall → PickObject (nova detecção)
```

O alvo do `FollowWall` é calculado a partir da última distância VL53 válida:

```text
travel_mm = 1000 * (preferred_tag_x_m - tag_x_m)
wall_mm = current_wall_mm + 1000 * (tag_y_m - preferred_tag_y_m)
```

`wall_mm` é limitado por `minimum_wall_distance_mm` e
`maximum_wall_distance_mm`. O destino lateral absoluto é limitado por
`minimum_lateral_position_mm` e `maximum_lateral_position_mm`, relativos à
posição `0`. Assim, se a centralização desejada ultrapassar uma extremidade, o
robô avança somente até o limite e repete a detecção nessa posição.
Deslocamento lateral positivo significa direita e negativo significa esquerda.

O deslocamento realmente medido pela action é acumulado na coordenada lateral,
em vez do valor comandado. Para uma tag vista anteriormente, o destino salvo é
convertido novamente em um deslocamento relativo à posição atual.

Para uma tag com posição salva, o robô vai ao alinhamento configurado e
obtém uma detecção nova, salvo a coleta direta imediatamente após a análise
explícita descrita acima. Se uma captura necessária após reposicionamento não
localizar a tag, a seleção considera outras tarefas identificadas antes de
continuar a busca. Uma tag desconhecida é procurada nos pontos ainda não
observados. As posições são coordenadas absolutas em milímetros, configuradas em
`pickup_recovery.search_positions_mm`; o padrão da arena é `[0, 325, -325]`.
Após esgotar a busca padrão, coleta e empilhamento tentam
`pickup_recovery.safety_search_positions_mm` à distância de parede
`pickup_recovery.safety_search_distance_mm`. A arena configura 60 mm e
`[-375, -250, -125, 0, 125, 250, 375]`. Primeiro a varredura usa LED ligado;
se o alvo continuar ausente, repete os sete pontos com LED apagado. O serviço
`/vision/hold_led_off` impede que cada análise religue o LED durante essa fase.
O LED volta a ligar ao terminar a busca, inclusive em falha ou cancelamento.
Uma lista de segurança vazia desativa as duas varreduras extras.

O histórico de segurança separa área, alinhamento, iluminação e detector.
Se X foi encontrado após quatro pontos de segurança iluminados, a busca de Y
que ainda não foi observado visita apenas os três pontos iluminados restantes,
seguindo para a varredura apagada se necessário. Todas as tags e contêineres
observados nessas fases continuam atualizando suas posições na memória.
Tentativas de movimento sem observação não contam como análise para outro passo.

Todas as posições de busca precisam estar dentro dos limites laterais.
Cada destino é marcado como tentado depois que o movimento termina, inclusive
quando `FollowWall` é interrompida por uma proteção tolerada. Se a proteção
impedir alcançar um destino lateral configurado, ele também é registrado como
bloqueado para as buscas de AprilTag e contêiner na mesma área. Esse registro
não afirma que o destino foi observado pela câmera: as observações de cada
detector continuam separadas, e a posição física usa somente o deslocamento
medido. Assim, a busca não volta a selecionar o extremo bloqueado.
Em cada posição, todas as outras tags encontradas também atualizam a memória.
Uma tag coletada é removida, sem apagar as demais observações. Se a próxima tag
não apareceu na última análise e a base continua na mesma posição, essa análise
não é repetida: o robô segue diretamente para o ponto de busca não examinado
mais próximo. Um ponto fixo só é marcado como examinado quando a distância da
parede também corresponde à distância de observação da área.

Os mesmos snapshots guardam containers por área e cor, assumindo nesta primeira
versão no máximo um container de cada cor por área. A memória preserva a
melhor observação completa e estável recebida durante a missão. Antes de um
`place_in_container`, ela serve apenas para retornar a base ao ponto onde o
container foi visto; a action sempre executa uma nova detecção antes de mover o
braço. Trocar de área não apaga observações das áreas anteriores.

## Recuperação de depósito

O passo `place_on_table` usa as posições de
`table_place_search_positions_mm`; na arena padrão são
`[0, 160, 325, -160, -325]`. Se o campo não for definido, usa as posições
de `pickup_recovery.search_positions_mm`. `place_in_container` continua usando
`pickup_recovery.search_positions_mm`. Se a visão não encontrar espaço livre
ou o contêiner solicitado, ou se a manipulação falhar antes de abrir a
garra, o mission manager mantém o objeto na garra, move a base para a posição
ainda não examinada mais próxima e repete a action semântica.
`place_in_container` reutiliza também as posições em que uma análise anterior
da missão já procurou containers e exclui destinos bloqueados durante a busca
de AprilTag. Se a cor solicitada não apareceu na última observação da posição
atual, não repete a mesma detecção. `place_on_table` mantém seu conjunto de
busca independente.

Contêiner e espaço livre também usam as duas varreduras de segurança antes
de esgotar a busca. A busca de espaço livre é renovada em cada depósito, pois
a ocupação da mesa pode mudar.

Depois de examinar as posições padrão e de segurança, o gerenciador chama
`PlaceOnTable` em modo de fallback. Nesse modo, a percepção é ignorada e a
posição padrão `x=0`, `y=-0,20` é usada com a altura, o offset do TCP, a
orientação preferencial e as restrições normais do perfil `table`. Se uma action
confirmar que a garra já foi aberta no destino e falhar somente no recuo ou no
retorno do braço, o efeito é aceito e o fluxo normal continua sem tentar
depositar o mesmo objeto outra vez. Resultado com efeito físico incerto continua
interrompendo a missão por segurança.

## Execução

```bash
ros2 launch mission_manager mission_manager.launch.py
```

```bash
ros2 action send_goal /mission/execute interfaces/action/ExecuteMission \
  "{plan_id: example_transport}" --feedback
```

Para executar o exemplo com contêineres e depósitos nas workspaces, migrado
pelos passos ativos do arquivo original:

```bash
ros2 action send_goal /mission/execute interfaces/action/ExecuteMission \
  "{plan_id: transportar_container_empilhar}" --feedback
```

Somente uma missão é aceita por vez. Falhas sem recuperação encerram a missão;
uma coleta não encontrada na posição atual retorna à seleção de tarefas. O
cancelamento é propagado para o goal filho ativo. O fallback de mesa existente
é preservado para depósitos em mesa/contêiner, com destino efetivo registrado.

## Estado do mundo

O `mission_manager` é o dono do estado lógico da garra e dos compartimentos.
Depois de validar o plano e a arena, cada nova missão reinicia esse estado como
conhecido, com garra e slots vazios. Antes de cada action física, o gerenciador
valida a precondição, preenche explicitamente o ID do objeto e somente depois
do resultado confirmado faz o commit da transição. Timeout, perda de comunicação,
cancelamento sem resultado ou efeito físico ambíguo tornam o estado desconhecido
e bloqueiam novas operações automáticas.

Para soltar o objeto que está na garra em um contêiner detectado na área atual,
use um passo como este em um plano YAML:

```yaml
- id: depositar_no_azul
  action: place_in_container
  tag_id: 1
  container_color: blue
```

`container_color` aceita `red` ou `blue`. O gerenciador envia a altura da área
de serviço e confirma a saída do objeto da garra somente quando a action relata
o depósito físico.

O snapshot atual é publicado em `/mission/state` com QoS `transient_local`.
Não existe uma API de estado usada pelo servidor de manipulação: `WorldState`
permanece interno ao gerenciador e é o ponto de extensão para incorporar
futuramente estados de objetos, estações e outros elementos da arena.

## Validação

Depois de compilar e carregar o workspace:

```bash
colcon build --packages-select interfaces manipulation mission_manager
source install/setup.bash
colcon test --packages-select mission_manager
colcon test-result --test-result-base build/mission_manager
```

O teste abaixo inicia o executor completo e servidores ROS simulados para
Nav2, FollowWall, visão e manipulação. Usa um domínio local separado; escolha um
`ROS_DOMAIN_ID` que não esteja em uso. Confirma 12 passos do exemplo
`test/fixtures/ros_visit_smoke.yaml`, transferências internas e a tag 3
armazenada ao sair da ws1. A fixture é separada dos planos usados no robô:

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/mission_visits_ros_logs \
  /usr/bin/python3 src/cbr_work/mission_manager/test/ros_visit_smoke.py
```

A execução simulada não substitui a calibração e o teste físico das trajetórias.
