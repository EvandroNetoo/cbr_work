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
- `place_on_shelf`: lógica implementada, bloqueada até medir a pose no SRDF;
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
altura da WS não faz parte da interface de empilhamento.

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

A coleta mantém o caminho `detect_apriltags` → `approach` → `grasp`. Depois de
fechar a garra, o MoveIt planeja explicitamente o retorno primeiro para
`approach` e depois para `detect_apriltags`. Não há ponto elevado adicional nem
reprodução de trajetórias armazenadas.

`place_on_table` sempre posiciona a câmera e solicita uma única sessão de
`/vision/analyze_scene` com AprilTags e containers. A grade nasce diretamente
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

`place_in_container` usa uma sessão do detector de contêineres, escolhe a única
detecção da cor solicitada e solta o objeto no centro do contorno externo. O TCP
usa X/Y da detecção com os offsets do perfil. Sua altura é
`ws_height_cm / 100 + external_height_m + reference_offset_xyz[2]`, sem usar o
Z visual. O braço faz um único movimento até a pose de soltura, restringindo a
posição e a orientação cartesiana para chegar com a garra reta e apontada para
baixo. Após abrir a garra, retorna diretamente para `detect_apriltags`, sem
pré-pose nem pose de recuo. A
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
  "{tag_id: 5, profile: tabletop}" --feedback
```

Armazenamento e retirada dos compartimentos calibrados:

```bash
ros2 action send_goal manipulation/store interfaces/action/StoreObject \
  "{object_tag_id: 5, slot_id: left}" --feedback
ros2 action send_goal manipulation/retrieve interfaces/action/RetrieveObject \
  "{object_tag_id: 5, slot_id: left}" --feedback
```

Para o compartimento direito, use os mesmos comandos com `slot_id: right`:

```bash
ros2 action send_goal manipulation/store interfaces/action/StoreObject \
  "{object_tag_id: 5, slot_id: right}" --feedback
ros2 action send_goal manipulation/retrieve interfaces/action/RetrieveObject \
  "{object_tag_id: 5, slot_id: right}" --feedback
```

Na retirada, `safe_state` é a pose segura de entrada e saída, enquanto
`retrieve_state` é a pose baixa onde a garra alcança o objeto. Para o
compartimento `left`, a sequência completa é `safe_cube_left` → `pre_grip` →
`pick_cube_left` → fechar em `grip` → `safe_cube_left`. No lado direito, a
mesma lógica usa `safe_cube_right` e `pick_cube_right`. O armazenamento usa
`store_state` (`deposit_cube_left/right`) para liberar o objeto; `home` fica
para a preparação da navegação da base.

Depósito em uma pose explícita do TCP (`arm_base_link`):

```bash
ros2 action send_goal manipulation/place_at_pose interfaces/action/PlaceAtPose \
  "{object_tag_id: 5, release_pose: {header: {frame_id: arm_base_link}, pose: \
    {position: {x: 0.20, y: 0.0, z: 0.10}, orientation: {w: 1.0}}}}" \
  --feedback
```

Interface para depósito automático em mesa:

```bash
ros2 action send_goal manipulation/place_on_table interfaces/action/PlaceOnTable \
  "{object_tag_id: 5, ws_height_cm: 12.5}" --feedback
```

Interface para depósito em contêiner:

```bash
ros2 action send_goal manipulation/place_in_container \
  interfaces/action/PlaceInContainer \
  "{object_tag_id: 5, ws_height_cm: 12.5, container_color: 1}" --feedback
```

Empilhamento sobre o cubo cuja AprilTag é 5:

```bash
ros2 action send_goal manipulation/stack interfaces/action/StackObject \
  "{object_tag_id: 3, support_tag_id: 5}" --feedback
```

Depósito na prateleira fixa:

```bash
ros2 action send_goal manipulation/place_on_shelf interfaces/action/PlaceOnShelf \
  "{object_tag_id: 5}" --feedback
```

O pacote não contém valores inventados para destinos ainda não medidos. Os
compartimentos internos `left` e `right` estão habilitados com poses existentes
no SRDF.
