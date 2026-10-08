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

Na aproximação de uma área de serviço, o braço vai para `detect_apriltags`
(`PrepareManipulator.OBSERVATION`) ao mesmo tempo que o alinhamento `FollowWall`.
Na saída, vai para `home` (`PrepareManipulator.NAVIGATION`, conforme as poses de
transporte em `manipulation/config/cargo_slots.yaml`) em paralelo ao recuo.
O executor espera os dois resultados antes da próxima ação. A falha ou o timeout
de uma ação não cancela a outra: o erro só é reportado após ambas terminarem.
O cancelamento solicitado pelo cliente continua interrompendo as ações. O estado da carga precisa ser conhecido. O Nav2
continua viajando com o braço na pose de transporte.

Antes do depósito em prateleira, a base alinha com `FollowWall` mantendo a pose
atual do braço. No retorno à distância de observação, o braço vai para
`detect_apriltags` em paralelo.

Guardar (`store`) e retirar (`retrieve`) dos slots também podem ocorrer enquanto
um `FollowWall` antecipa o próximo reposicionamento ou recuo de saída. A base só
é liberada pelo feedback `APPROACHING`, após a preparação segura do braço. Para
slot `left`, somente deslocamento para a direita; para slot `right`, somente
para a esquerda. O executor usa o slot escolhido e o deslocamento efetivo após
os limites laterais e o recuo de folga já existentes. Mesmo lado, deslocamento
sem componente lateral, slot sem lado conhecido ou destino ainda não decidido
mantêm a execução sequencial.

O destino vem da próxima escolha viável: memória de cubo/suporte/contêiner,
próximo ponto de uma busca já iniciada ou recuo configurado para a próxima visita.
A saída antecipada não repete o recuo. Outra ação do braço e o Nav2 esperam os dois
resultados, e a carga só é atualizada pelo resultado físico confirmado da transferência.
Durante a sobreposição, a recuperação lateral automática fica desativada para
não inverter o sentido em direção ao slot. Paradas de proteção reconhecidas com
sensores e odometria válidos mantêm a base parada e permitem concluir a transferência;
o executor conserva a posição medida e registra destinos de busca bloqueados.
Uma saída parcial não é marcada como concluída. Falhas de comunicação, sensores
ou manipulação são reportadas após a outra ação terminar. Cada ação mantém seu
próprio timeout e suas proteções locais; o executor não cancela sua parceira.

## Formato de missão v2

A ordem de `visits` define a rota. Dentro de cada visita, a ordem de `tasks`
não determina a execução: o executor considera identificação, carga e viabilidade
de concluir todas as visitas restantes. Tags não solicitadas são ignoradas.

```yaml
schema_version: 2
plan_id: coleta_flexivel
finish: true
visits:
  - target: ws_1
    tasks:
      - {action: pick, tag_id: 1}
      - {action: pick, tag_id: 2}
      - {action: pick, tag_id: 3}
  - target: ws_2
    tasks:
      - {action: place_on_table, tag_id: 1}
      - {action: place_in_container, tag_id: 2, container_color: red}
      - {action: pick, tag_id: 4}
  - target: ws_3
    tasks:
      - {action: place_on_table, tag_id: 3}
      - {action: place_on_table, tag_id: 4}
```

IDs de visitas e tarefas são gerados automaticamente; não é necessário informar
`id`. A visita usa o local (`visit_ws_1`); a tarefa acrescenta ação e parâmetros
(`visit_ws_1_pick_1`, `visit_ws_2_place_in_container_2_red`,
`visit_ws_4_stack_4_5_on_14`). Visitas repetidas e colisões recebem sufixos
`_2`, `_3` etc. Os IDs aparecem no feedback e identificam falhas. IDs explícitos
continuam aceitos e precisam ser únicos. `plan_id` continua obrigatório para
selecionar a missão pela action ROS. `tasks: []` é
permitido para visitas de navegação. `initial_location` tem padrão `start` e
informa a localização física inicial; cada visita ainda executa sua navegação.
`finish` tem padrão `false`; quando verdadeiro, navega ao ponto `finish` após
concluir as visitas. A arena permanece em `schema_version: 1`.

Cada coleta ou entrega informa `tag_id`. As entregas disponíveis são
`place_on_table`, `place_in_container` (`container_color: red|blue`),
`place_on_shelf` e `stack`. Uma pilha sem ordem fixa é declarada assim:

```yaml
- action: stack
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
Exemplos inconsistentes não são instalados como planos executáveis. O advanced tem uma versão
v2 executável em `config/plans/advanced_transportation_test_i.yaml`, com a pilha
de 4 e 5 sobre 14 na ws_5 e a entrega de 3 na ws_1. `simples` não está entre
os planos instalados porque não declarava a origem da carga.

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
goals; o valor no YAML fornecido é 1,0 s (2,0 s no padrão do nó).
Cada mesa pode sobrescrever essa janela com
`service_areas.<mesa>.alignment_error_ignore_sec` no `arena.yaml`. O valor
deve ser finito e não negativo; `0` aplica o limite imediatamente. Se omitido,
usa o parâmetro global. A janela da mesa vale para todos os seus movimentos
laterais, inclusive busca, reposicionamento e saída.
Durante o alinhamento frontal de chegada, os três
campos são enviados como `0`; no recuo de uma mesa, os limites do bloco
`departure` são usados quando houver retorno lateral, com a janela
configurada para a mesa ou, quando omitida, no mission manager. Aborto
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
abre a garra e retorna diretamente a `detect_apriltags`. O tipo `SH` não troca a ação do plano
automaticamente, e `height_cm` não define a altura desse depósito.

O plano `example_shelf` demonstra coleta e depósito na `sh_1`. Os valores de
`sh_1` em `config/arena.yaml` e as juntas de `place_on_shelf_high` no arquivo
`so_arm_101_moveit_config/config/so_arm_101.srdf` são **fictícios** e devem ser
substituídos pela calibração antes de executar no robô. O perfil `shelf` de
`manipulation/config/profiles.yaml` já aponta para essa pose e está habilitado.

## Recuo lateral antes de acessar o slot

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
Antes de `RetrieveObject`, a mesma verificação usa a folga no lado do slot
fornecida pelo último alinhamento, mesmo sem deslocamento lateral prévio ou
quando o último deslocamento foi para o outro lado. Se estiver abaixo do
mesmo limite, a base se afasta do lado do slot pelo mesmo percurso configurado
antes de o braço retirar o cubo. Uma leitura obsoleta, um limite de posição
que impeça o recuo completo ou uma falha no movimento bloqueia a retirada.
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

Antes de qualquer depósito em uma área `SH`, o gerenciador executa `FollowWall`
para confirmar a distância frontal do VL53, mantendo a pose atual do braço.
A ação de depósito começa após esse alinhamento, sem preparação em `home`.
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
- action: place_in_container
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

### Entrega na mesa de precisão (PP)

Declare a mesa com `type: PP` em `config/arena.yaml`. A tarefa distingue a tag
do objeto transportado (`tag_id`) da tag fixa da mesa (`reference_tag_id`):

```yaml
schema_version: 2
plan_id: precision_delivery
visits:
  - target: ws_1
    tasks:
      - {action: pick, tag_id: 5}
  - target: pp_1
    tasks:
      - {action: place_on_precision_table, tag_id: 5, reference_tag_id: 42}
finish: true
```

`pp_1` deve existir na arena como área PP. A tag 42 é uma referência da mesa e
não entra no inventário de carga. As observações de AprilTags encontradas na
mesa são armazenadas pela mesma lógica usada nas coletas e no stack. Para a
entrega, o gerenciador calcula o destino a partir da pose memorizada e dos
alvos X/Y específicos do PP, indo diretamente à posição preferida em um único
reposicionamento. Ao confirmar a chegada, detecta a referência novamente para
calcular a pose de soltura. Se a chegada ficar incompleta, solicita alinhamento
pela nova detecção; se a tag não aparecer, mantém a recuperação e a busca.
Sem uma referência memorizada, detecta primeiro e depois alinha a base.

Os alvos de alinhamento são `pickup_recovery.precision_preferred_tag_x_m` e
`precision_preferred_tag_y_m`, independentes do stack. Calibre esses alvos junto
com `placements.precision_table.reference_offset_xyz` no pacote `manipulation`,
para que a cavidade fique ao alcance do braço. O offset é somado em metros no
referencial `arm_base_link`. Antes de executar, configure o offset real e
marque `calibrated_reference: true` no perfil PP.

### Organizar os alojamentos PP (Advanced Manipulation Test)

Declare apenas o destino desejado. O estado inicial e as cavidades vazias são
observados durante a execução; a mesa possui sete cavidades, inclusive quando
as sete estão ocupadas:

```yaml
schema_version: 2
plan_id: cubos_1_2_3
finish: false
visits:
- target: pp_1
  tasks:
  - final_state: {1: 1, 2: 2, 3: 3, 5: 4, 6: 5, 4: 6}
```

Cada chave identifica a AprilTag fixa de uma cavidade; o valor identifica a
AprilTag do objeto destinado a ela. Referências e objetos podem usar o mesmo ID.
`null` exige a cavidade vazia. Cavidades omitidas não têm ocupação final prescrita
e podem receber objetos excedentes. Objetos já confirmados no destino ficam no
lugar. IDs de objetos devem ser únicos dentro de `final_state`.

Somente em PP, a visão preserva duas detecções do mesmo ID, uma por função.
O gerenciador mantém memórias separadas para objetos e referências. O pick recebe
somente objetos; o place recebe somente referências. A função é determinada
pelo Z detectado em **arm_base_link**:

```yaml
# config/arena.yaml — valores de exemplo, calibrar no robô real
precision_perception:
  reference_z_m: 0.02
  reference_z_tolerance_m: 0.015
  slot_offset_x_m: 0.0
  slot_offset_y_m: 0.0675
  occupancy_radius_m: 0.025
```

Uma tag na faixa `[reference_z_m - reference_z_tolerance_m,
reference_z_m + reference_z_tolerance_m]` é referência, incluindo os limites;
fora dela é objeto. Meça a altura das referências reais neste referencial e
ajuste a tolerância para separar os cubos. Essas alturas não são a altura da mesa
em relação ao chão. WS e SH mantêm sua classificação anterior.

Os offsets XY indicam o centro da cavidade em relação à referência e devem
coincidir com XY de `placements.precision_table.reference_offset_xyz` em
`manipulation/config/profiles.yaml`. O raio XY determina quando um objeto
observado ocupa a cavidade; calibre-o para distinguir cavidades vizinhas.
A referência precisa aparecer em uma análise válida na posição atual para
considerar o slot vazio. Ausência da referência, frames sem TF e detecção
ambígua interrompem a operação em vez de presumir uma cavidade livre.

O fluxo prioriza objetos observados fora do destino e objetos já armazenados.
Cada nova análise, inclusive durante a busca de um objeto, reconhece os pares
referência/objeto já corretos e os marca como concluídos. Antes do pick, essa
decisão é reavaliada com a imagem disponível: se o objeto procurado já estiver
em seu destino, a coleta é dispensada e a organização segue para os pendentes.
Isso não exige uma foto adicional nem conhecer a referência antes de buscar o
cubo. Um slot ainda não observado não é considerado vazio nem incorreto.
A referência de destino não precisa ser conhecida antes da coleta: ela é
procurada depois de coletar e armazenar o cubo. Referências já memorizadas permitem
ir diretamente à posição preferida, com uma única análise da ocupação no destino.
Se ocupada, a pose dessa mesma análise é usada para coletar o ocupante, sem nova
análise ou alinhamento de coleta. Depois: armazenar o ocupante → recuperar o cubo
correto → analisar e depositar na própria action de place. O ocupante deslocado
é o próximo a organizar. Não há análise intermediária para confirmar que a
cavidade esvaziou, nem análise após o depósito para confirmar a tag colocada.
O resultado físico de coleta/soltura atualiza o estado esperado dos slots.

A posição de alinhamento é reaproveitada no place. Só é necessário retornar à
referência se a base tiver mudado ao acessar a carga; falhas de alcance mantêm
os mecanismos de recuperação. A análise final no próprio place localiza a
referência e recusa liberar sobre outro objeto antes de abrir a garra.

Se essa análise final detectar ocupação que não apareceu antes, a organização
continua: o cubo na garra volta ao compartimento liberado pelo retrieve, a
ocupação atualiza o planejamento, o ocupante é retirado e armazenado e o depósito
é tentado novamente. Com os dois compartimentos cheios, primeiro se deposita um
cubo em seu destino vazio; se ambos os destinos estiverem ocupados, uma cavidade
vazia pode servir de apoio temporário, respeitando o ciclo store/retrieve. Esse
apoio não conta como conclusão do slot: o cubo será levado ao destino final.
Os slots que a análise contradizer deixam de ser considerados concluídos.

Falhas transitórias de percepção, movimento, servidor ocupado ou indisponível
no place PP permitem até duas novas tentativas, somente se o resultado confirmar
o cubo ainda na garra. Pedidos repetidos de alinhamento também têm recuperação
limitada. Recuperações de ocupação podem replanejar dentro de até 84 iterações
da organização; falhas persistentes e estado físico desconhecido continuam
interrompendo a missão, preservando o inventário.

Todo cubo passa por armazenamento e retirada antes de cada depósito PP.
A visita requer garra vazia e pelo menos dois compartimentos internos livres;
isso resolve ciclos mesmo com a mesa cheia. Um objeto sem destino pode ficar
armazenado enquanto a cadeia avança para uma cavidade livre. Ao terminar, objetos
excedentes usam cavidades omitidas disponíveis ou permanecem na carga interna,
respeitando a capacidade. A carga restante é considerada nas próximas visitas da mesma missão. Uma nova
missão não apaga o inventário PP remanescente: exige recuperação dessa carga.
Não se usa uma zona livre da mesa ou um contêiner como apoio. Os movimentos de
rebolada configurados em `placements.precision_table.base_wiggle` continuam
antes da abertura da garra.

A busca de referências e objetos usa `pickup_recovery.search_positions_mm`
e, depois, `safety_search_positions_mm` na distância de segurança. Se uma
referência memorizada não aparecer depois do alinhamento, o gerenciador busca
nessas posições antes de falhar. O mesmo vale para um ocupante que sumiu da
análise e para tags ausentes nas actions de pick/place em PP. Para uma referência
ainda não localizada, a organização reaproveita as análises válidas da visita:
primeiro busca em posições onde sua presença ainda não foi verificada. Pegar e
armazenar um cubo invalida a ocupação, mas mantém o histórico de referências
fixas. Exemplo: após analisar 0 e 325 sem ver a referência, busca primeiro em
-325, sem repetir a análise em 325 nem voltar antes a 0. O histórico distingue
área, distância frontal, posição lateral, iluminação e IDs de referências
(independentes dos IDs de objetos). Se todas as posições restantes falharem,
revisita as negativas antigas uma vez: uma coleta pode revelar uma tag antes
encoberta. Referências já conhecidas que sumirem após alinhamento mantêm busca
própria, e a ocupação do destino continua exigindo observação atual.
Só a busca esgotada causa falha por
tag ausente; cancelamento, falhas de percepção e incerteza física continuam
interrompendo a execução. A carga física permanece registrada. O número de operações é decidido durante a
execução; o total no feedback começa como estimativa e cresce se necessário.

O formato antigo com `start_state` e `final_state` continua aceito e usa o
planejador da distribuição declarada. Esses dois mapas precisam ter os mesmos
slots e objetos. A organização exige `type: PP`, offset calibrado e não pode ser
misturada com actions explícitas dentro da mesma visita.
