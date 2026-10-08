# manipulation

Servidor ROS 2 que transforma MoveIt e percepção em ações semânticas. Os
objetos são sempre coletados sobre uma mesa. Cada tipo de depósito possui uma
interface própria; a antiga action genérica `PlaceObject` foi removida.

## Actions

```text
manipulation/pick
manipulation/store
manipulation/retrieve
manipulation/place_on_table
manipulation/place_in_container
manipulation/stack
manipulation/place_on_shelf
manipulation/place_at_pose
manipulation/prepare
```

Situação atual dos depósitos:

- `place_at_pose`: funcional, para calibração, testes e poses explícitas;
- `stack`: lógica implementada e habilitada com o offset configurado no perfil;
- `place_on_shelf`: habilitado com a pose alta fixa `place_on_shelf_high`;
  as juntas no SRDF são fictícias e precisam ser calibradas antes do uso no robô;
- `place_on_table`: depósito disponível após preencher yaw, offset e região;
  análise de obstáculos por AprilTags disponível após calibrar a região de busca;
- `place_in_container`: habilitado para soltar no centro do contêiner detectado
  da cor solicitada;
- mesa de precisão: deliberadamente fora do escopo atual.

Todas as actions que movem um objeto recebem seu ID explicitamente.
`manipulation` não consulta, valida nem altera o estado da missão: executa o
comando físico recebido e informa no resultado se o efeito sobre a carga ficou
conhecido. O `mission_manager` é responsável por autorizar a operação antes do
envio e atualizar seu inventário depois do resultado.
No empilhamento, `support_tag_id` identifica apenas o cubo de apoio.
As coordenadas X, Y e Z do apoio são obtidas da pose 3D dessa AprilTag; a
altura da WS é enviada para permitir a detecção opcional de containers parciais
na mesma sessão visual.

As análises retornam um `SceneObservation` comum com todas as AprilTags e
containers observados. A seleção é configurada por operação em
`vision_detectors.*`: por padrão `pick` e `stack` analisam AprilTags e
contêineres HSV juntos, `place_in_container` analisa o contêiner HSV e
`place_on_table` solicita a superfície da mesa e, por padrão, as AprilTags
na mesma sessão. A máscara da superfície aceita branco e preto; a seleção
combina a grade livre com um disco de exclusão ao redor de cada tag. A flag
`table_apriltag_blocking_enabled` (padrão: `true`) desativa tanto a análise
de AprilTags nessa operação quanto esse bloqueio quando vale `false`.
O raio do disco é `table_apriltag_clearance_radius_m` (padrão: 0,02 m).

O `PickObject` pode receber `use_observed_detection: true` e uma
`observed_detection` obtida na análise explícita imediatamente anterior à
coleta, sem nenhuma operação ou deslocamento intermediário.
Nesse modo, não posiciona a câmera nem chama análise de cena; valida ID,
referencial `arm_base_link`, valores finitos e quaternion antes de agir, e
aplica o mesmo filtro de alcance e planejamento do perfil. O resultado indica
`used_observed_detection: true`; isso não representa uma nova captura. O
cliente deve usar esse modo apenas na ação imediatamente após a análise.
Manipulação, navegação ou reposicionamento consomem essa autorização; voltar ao
mesmo ponto não a renova. Uma posição memorizada exige `false` e uma nova análise
após o alinhamento configurado. Chamadas isoladas
mantêm `false` por padrão. Tentativas internas no mesmo ponto reutilizam a
primeira captura, sem repetir fotografias a cada tentativa de planejamento.

Quando `pickup.tabletop.reachability_filter_enabled` está habilitado, a coleta
usa seus próprios limites `reach_x/y_*`, CP e CL, definidos em
`pickup.tabletop`. Uma AprilTag detectada fora dessa região é rejeitada antes
do planejamento cartesiano e não consome uma segunda tentativa. Desabilitar a
flag preserva o envio direto da pose ao MoveIt.
Nos bloqueios do filtro e nas falhas MoveIt de código `99999`, o resultado de
`PickObject` informa `recovery_reason` e `detected_pose` para que o gerenciador
de missão possa reposicionar a base antes de repetir a detecção.
O mesmo resultado sempre inclui `observed_detections`, com a melhor observação
de cada ID encontrado nas tentativas executadas sem movimentar a base. Isso
também vale para coleta bem-sucedida e para alvo não encontrado.

A coleta superior mantém o caminho `detect_apriltags` → `approach` → `grasp`. Depois de
fechar a garra, o MoveIt planeja explicitamente o retorno primeiro para
`approach` e depois para `detect_apriltags`. Não há ponto elevado adicional nem
reprodução de trajetórias armazenadas.

O perfil `pickup.shelf_front` usa pegada frontal.
O filtro `reachability_filter_enabled` desse perfil fica desabilitado: depois
da tentativa de alinhamento, a pose segue diretamente ao MoveIt, sem limites
de raio ou XY da coleta. A primeira chamada retorna
`RECOVERY_ALIGNMENT_REQUIRED` com a pose detectada, antes de mover ao cubo.
Após tentar centralizar a base, o gerenciador envia `alignment_completed`.
A ação detecta novamente e aceita essa posição, mesmo fora das tolerâncias
de centralização. Então executa `home` → `pre_grasp_state`, vai diretamente à posição
do cubo e fecha a garra. Depois retorna na ordem inversa da preparação:
`pre_grasp_state` → `home`, encerrando o pick em `home`, sem pose adicional
de retreat nem retorno a `detect_apriltags`.
Se o MoveIt falhar com código `99999` antes de fechar a garra, a coleta
frontal retorna a `home` antes de solicitar a recuperação da base. Só depois
desse retorno a missão pode preparar `detect_apriltags` e repetir a detecção.
Se o retorno a `home` falhar, a recuperação é interrompida.
O Z do TCP é o topo da tag menos metade de `cube_size_m`, acrescido de
`grasp_z_offset_m`. O Y do TCP é o Y detectado mais `grasp_y_offset_m`;
um offset positivo traz o alvo para o lado do robô.
O alvo de pegada restringe somente a posição do TCP e
`link3_to_link4_deg` e `link4_to_link5_deg`, ambas com
`joint_tolerance_deg`, sem orientação cartesiana obrigatória,
ponto intermediário de aproximação ou restrições de trajeto. O MoveIt calcula
livremente a rota até esses alvos. `yaw_offset_deg` não tem função na pegada
frontal e foi removido de `pickup.shelf_front`; permanece na coleta superior,
onde gira a orientação calculada a partir da tag. `approach_height_m` também
é usado somente na estratégia superior.

`pick_shelf_front_ready` no SRDF define a pose inicial ajustável da SH.
Na ida e na volta, sua junta `link4_to_link5` é sobrescrita pelo valor de
`pickup.shelf_front.link4_to_link5_deg`; as demais juntas mantêm os valores
do SRDF. Essa cópia não altera a definição original do estado.
Calibre essa pose e o offset Z no robô; o planejamento precisa
encontrar soluções para os alvos. A prateleira precisa estar na Planning Scene para
verificação de colisão de todos os elos. Nenhum teste físico foi executado.

`place_on_table` sempre posiciona a câmera e solicita uma única sessão de
`/vision/analyze_scene` com a superfície da mesa e, se a flag estiver ativa,
com as AprilTags. A superfície é obrigatória; a flag controla as tags mesmo
que `vision_detectors.place_on_table` contenha `apriltags`. A grade nasce diretamente
dos limites `search_x_min_m`, `search_x_max_m`, `search_y_min_m` e
`search_y_max_m`, usando `search_step_m`; a altura do TCP é calculada por
`(ws_height_cm + tcp_release_offset_cm) / 100`. Os candidatos válidos para o
alcance são embaralhados antes da busca. Em cada candidato, a busca testa
`free_space_preferred_yaw_deg` e depois `free_space_alternate_yaw_deg`. A área
ocupada pela garra é um retângulo
orientado, com meias dimensões `free_space_half_extent_x_m` e
`free_space_half_extent_y_m`; os eixos desse retângulo giram junto com o yaw.
Primeiro, `free_space_preferred_padding_m` é somado aos quatro lados. Se nenhum
candidato passar, a busca tenta novamente com `free_space_min_padding_m`, que
nunca é removido e representa a folga obrigatória.
As AprilTags, exceto a do objeto na garra, são testadas contra esse retângulo.
O footprint externo de cada container é incluído como outro retângulo orientado
pelas detecções. Contornos completos e cortados pela borda da imagem usam o
mesmo retângulo externo estimado durante a busca de espaço livre.
A busca usa uma grade delimitada por `search_x_min_m`, `search_x_max_m`,
`search_y_min_m` e `search_y_max_m`. Essa grade é recortada pela faixa circular
centrada em `reach_center_x_m/reach_center_y_m`: pontos abaixo de
`reach_min_radius_m` (CP) ou acima de `reach_max_radius_m` (CL) são descartados.
Os candidatos alcançáveis são embaralhados antes dos testes; se nenhum candidato
for livre, a action retorna `NO_FREE_SPACE` sem iniciar o depósito.

`place_in_container` usa uma sessão combinada de visão, escolhe a única
detecção da cor solicitada e solta o objeto no centro do contorno externo. O TCP
usa X/Y da detecção com os offsets do perfil. Sua altura é
`ws_height_cm / 100 + external_height_m + reference_offset_xyz[2]`, sem usar o
Z visual. O objetivo MoveIt restringe a posição do TCP e mantém a junta
`link4_to_link5` em −90° com tolerância de ±5°, sem impor orientação cartesiana
ao TCP. Também limita `link3_to_link4` ao máximo configurado em
`placements.container.link3_to_link4_max_deg` (−10° por padrão). A orientação
neutra em `placed_pose` não representa o ângulo real
alcançado. O braço vai diretamente à pose de soltura, sem aproximação. Após
abrir a garra, volta diretamente para `detect_apriltags`, sem pose de recuo. A
action rejeita ausência, duplicidade e geometria inválida antes do movimento ao
destino. A abertura interna não é medida pelo detector; conferir no robô se a
pose central e a altura permitem a queda do cubo.
Uma detecção parcial na borda também pode ser escolhida como destino; o
feedback informa a incerteza XY estimada antes do movimento. O perfil
`placements.container` define os limites de sobreposição do ajuste e
incerteza XY para aceitar essa pose parcial como alvo.

Os demais depósitos cartesianos elevam o braço após liberar o objeto e então
seguem para `detect_apriltags`. O retorno para `home` fica a cargo de
`PrepareManipulator` no modo `NAVIGATION`, evitando o desvio por `home` quando a
próxima operação também acontece na mesa.

O servidor aceita somente uma operação por vez e propaga cancelamento para o
goal ativo do MoveIt ou do detector. Após cancelar, o braço permanece parado;
nenhum movimento automático de recuperação é iniciado. O servidor não possui
inventário nem cliente do `mission_manager`. O tópico `/mission/state` é uma
saída exclusiva do gerenciador e usa durabilidade `transient_local`.

## Execução

O servidor também pode ser iniciado sem o `mission_manager`; nesse caso o
chamador assume integralmente as verificações de segurança e deve preencher
todos os campos das actions. Inicie o MoveIt, a câmera e o detector de
AprilTags. Depois:

```bash
ros2 launch manipulation manipulation.launch.py
```

Coleta do objeto 5 sobre a mesa:

```bash
ros2 action send_goal manipulation/pick interfaces/action/PickObject \
  "{tag_id: 5, profile: tabletop, ws_height_cm: 12.5}" --feedback
```

Armazenamento e retirada dos compartimentos calibrados:

```bash
ros2 action send_goal manipulation/store interfaces/action/StoreObject \
  "{slot_id: left}" --feedback
ros2 action send_goal manipulation/retrieve interfaces/action/RetrieveObject \
  "{slot_id: left}" --feedback
```

Para o compartimento direito, use os mesmos comandos com `slot_id: right`:

```bash
ros2 action send_goal manipulation/store interfaces/action/StoreObject \
  "{slot_id: right}" --feedback
ros2 action send_goal manipulation/retrieve interfaces/action/RetrieveObject \
  "{slot_id: right}" --feedback
```

Toda retirada começa pela pose de observação `detect_apriltags`, configurada
em `pickup.tabletop.observation_state`, antes de entrar no compartimento.
Na retirada, `safe_state` é a pose segura de entrada e saída, enquanto
`retrieve_state` é a pose baixa onde a garra alcança o objeto. Para o
compartimento `left`, a sequência completa é `detect_apriltags` → (`safe_cube_left` + `pre_grip`, juntos) →
`pick_cube_left` → fechar em `grip` → `safe_cube_left`. No lado direito, a
mesma lógica usa `safe_cube_right` e `pick_cube_right`. Braço e garra usam
um único goal do grupo MoveIt `arm_gripper`; a descida aguarda o término dos dois. O armazenamento usa
`store_state` (`deposit_cube_left/right`) para liberar o objeto; `home` fica
para a preparação da navegação da base.

Depósito em uma pose explícita do TCP (`arm_base_link`):

```bash
ros2 action send_goal manipulation/place_at_pose interfaces/action/PlaceAtPose \
  "{release_pose: {header: {frame_id: arm_base_link}, pose: \
    {position: {x: 0.20, y: 0.0, z: 0.10}, orientation: {w: 1.0}}}}" \
  --feedback
```

Interface para depósito automático em mesa:

```bash
ros2 action send_goal manipulation/place_on_table interfaces/action/PlaceOnTable \
  "{ws_height_cm: 12.5}" --feedback
```

O campo `use_fallback_pose` é reservado ao `mission_manager`: após esgotar as
posições de observação, ele mantém a action `PlaceOnTable`, ignora uma nova
análise da cena e usa a posição padrão com o perfil normal de depósito na mesa.

Interface para depósito em contêiner:

```bash
ros2 action send_goal manipulation/place_in_container \
  interfaces/action/PlaceInContainer \
  "{ws_height_cm: 12.5, container_color: 1}" --feedback
```

Empilhamento sobre o cubo cuja AprilTag é 5:

O `mission_manager` usa `StackObject.require_alignment` para solicitar somente
a localização do apoio antes de mover a base. A action retorna
`RECOVERY_ALIGNMENT_REQUIRED` e a pose detectada, sem mover à soltura nem abrir
a garra. Após o alinhamento, o gerenciador envia `require_alignment: false`, e a
action detecta novamente antes de empilhar. Comandos diretos mantêm o padrão
`require_alignment: false` e executam o stack normalmente.

A tolerância de inclinação na soltura é configurada em
`placements.stack.tilt_tolerance_deg` no `config/profiles.yaml`, em graus.
Por exemplo, `5.0` permite até 5 graus de inclinação nos eixos X e Z da
restrição de orientação. Omitir o campo ou usar `null` mantém o padrão
de 0.20 radianos (aproximadamente 11.46 graus). O valor deve ficar entre 0 e 180.
A aproximação e a retirada mantêm a tolerância de inclinação de 35 graus,
e a tolerância de giro (yaw) permanece com o padrão global de 5 graus.

Nos depósitos que usam aproximação e recuo por pose (`stack`, `table` e
`explicit_pose`), `approach_height_m: 0` pula o movimento de aproximação
elevada e segue diretamente à pose de soltura. `retreat_height_m: 0` pula a
elevação após soltar e segue ao retorno para `detect_apriltags`. Cada campo
pode ser zerado independentemente; as fases puladas não publicam feedback
de aproximação ou elevação. Valores negativos não são aceitos.

```bash
ros2 action send_goal manipulation/stack interfaces/action/StackObject \
  "{support_tag_id: 5, ws_height_cm: 12.5}" --feedback
```

Depósito na prateleira fixa:

```bash
ros2 action send_goal manipulation/place_on_shelf interfaces/action/PlaceOnShelf \
  "{}" --feedback
```

O perfil `shelf` está habilitado e usa `place_on_shelf_high` no SRDF. Essa
pose contém valores **fictícios** em radianos: substitua-os pela calibração
antes de executar no robô. A action não recebe altura nem pose de destino. Após liberar o objeto, retorna
diretamente à pose de observação `detect_apriltags`, sem passar por `home`.
Os compartimentos internos `left` e `right` mantêm suas poses medidas no SRDF.

### Depósito na mesa de precisão (PP)

A ação `manipulation/place_on_precision_table` (`interfaces/action/PlaceOnPrecisionTable`)
recebe `reference_tag_id`: a AprilTag fixa que referencia a cavidade. O perfil
`placements.precision_table`, em `config/profiles.yaml`, configura o depósito:

```yaml
precision_table:
  strategy: tag_relative
  enabled: true
  calibrated_reference: true  # Somente após medir e validar o offset real.
  reference_offset_xyz: [0.03, -0.04, 0.05]  # Exemplo ilustrativo, em metros.
  yaw_offset_deg: 0.0
  approach_height_m: 0.08
  retreat_height_m: 0.08
```

A pose detectada é transformada para `arm_base_link`. A pose de soltura do TCP é
`XYZ_tag + reference_offset_xyz`, somando nos eixos do braço; o offset não gira
com os eixos locais da tag. Z inclui o ajuste necessário da cavidade até o TCP
com o objeto preso. O yaw segue a normalização usada no stack, acrescida de
`yaw_offset_deg`. Aproximação e retirada usam as alturas do perfil PP.

O depósito exige `calibrated_reference: true`. Calibre a relação entre a tag e
a pose de soltura antes de habilitar depósitos reais; use `false` enquanto a
calibração estiver pendente. Essa calibração é independente do stack. Uma tag
ausente não permite a soltura.

```bash
ros2 action send_goal manipulation/place_on_precision_table interfaces/action/PlaceOnPrecisionTable \
  '{reference_tag_id: 42, ws_height_cm: 15.0, require_alignment: false}'
```

O Mission Manager usa `require_alignment: true` na primeira chamada, alinha a
base a partir da pose detectada e solicita uma nova detecção antes do depósito.
A seleção de detectores é `vision_detectors.place_on_precision_table: [apriltags]`.


### Armazenamento seguido de retirada imediata

`StoreObject` aceita `prepare_retrieve: true` para terminar com o braço em
`safe_state` do compartimento e a garra em `pre_grip`. Uma retirada imediata do
mesmo slot usa essa preparação para seguir a `retrieve_state`, fechar a garra
e retornar a `safe_state`, sem passar por `detect_apriltags` entre as actions.
Qualquer outra operação invalida a preparação. O valor padrão `false` mantém o
armazenamento independente com retorno a `detect_apriltags`.

O Mission Manager usa essa opção nos pares consecutivos `store → retrieve`.
Em missões PP, cada coleta exige esse ciclo antes do depósito para padronizar a
posição do cubo na garra.

### Rebolada da base antes da soltura na PP

`placements.precision_table.base_wiggle` configura uma oscilação circular
temporizada da base, depois de atingir a pose de soltura e antes de abrir a
garra. O braço mantém a pose e a garra permanece fechada durante o movimento.

```yaml
base_wiggle:
  enabled: true
  radius_m: 0.005
  cycles: 2
  period_s: 1.5
  max_speed_m_s: 0.02
  settle_s: 0.3
  rate_hz: 30.0
```

A amplitude cresce suavemente e diminui até zero. São publicados comandos
`TwistStamped` em `base_wiggle.cmd_vel_topic: /cmd_vel`, usando
`base_wiggle.command_frame: base_footprint`. A duração é `cycles * period_s`,
seguida de velocidade zero e uma pausa fixa de `settle_s` antes da soltura.
Se a velocidade nominal exceder `max_speed_m_s`, toda a trajetória é escalada
uniformemente; nesse caso, o raio comandado também diminui.

Não há leitura de odometria nem verificações de movimento inicial, velocidade
medida, acompanhamento, deslocamento, retorno, orientação ou assinantes do
tópico. O término depende apenas do tempo; o retorno físico à posição inicial
não é confirmado. Os parâmetros antigos de feedback são aceitos e ignorados.
As verificações de configuração se limitam a tipos e valores utilizáveis.

A parada em velocidade zero é enviada ao terminar, cancelar ou falhar. O
cancelamento padrão da action continua ativo e impede a abertura da garra.
Não há controle de força nem ajuste automático de Z. Para desativar a etapa,
use `enabled: false`. Durante a rebolada, a manipulação controla a base;
o fluxo normal do Mission Manager não executa FollowWall simultaneamente.
