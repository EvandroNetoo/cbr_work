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
- `place_on_table`: depósito nominal disponível após preencher X/Y/yaw/offset;
  análises de obstáculos ainda aguardam os respectivos planejadores;
- `place_in_container`: interface pronta, aguardando o detector de contêineres;
- mesa de precisão: deliberadamente fora do escopo atual.

Somente `pick` recebe o ID do objeto carregado. `store`, `retrieve` e os
depósitos inferem o objeto pelo inventário publicado em `manipulation/state`.
No empilhamento, `support_tag_id` identifica apenas o cubo de apoio.
As coordenadas X, Y e Z do apoio são obtidas da pose 3D dessa AprilTag; a
altura da WS não faz parte da interface de empilhamento.

Quando `analyze_apriltags` e `analyze_containers` são falsos,
`place_on_table` usa `release_x_m`, `release_y_m` e `release_yaw_deg` fixos do
perfil `table`. A altura do TCP é calculada por
`(ws_height_cm + tcp_release_offset_cm) / 100`.

O servidor aceita somente uma operação por vez e propaga cancelamento para o
goal ativo do MoveIt ou do detector. Após cancelar, o braço permanece parado;
nenhum movimento automático de recuperação é iniciado. O estado lógico da garra e dos
compartimentos é publicado em `manipulation/state` com durabilidade
`transient_local`.

## Execução

Inicie antes o MoveIt, a câmera e o detector de AprilTags. Depois:

```bash
ros2 launch manipulation manipulation.launch.py
```

Coleta do objeto 5 sobre a mesa:

```bash
ros2 action send_goal manipulation/pick interfaces/action/PickObject \
  "{tag_id: 5, profile: tabletop}" --feedback
```

Armazenamento e retirada do compartimento calibrado:

```bash
ros2 action send_goal manipulation/store interfaces/action/StoreObject \
  "{slot_id: left}" --feedback
ros2 action send_goal manipulation/retrieve interfaces/action/RetrieveObject \
  "{slot_id: left}" --feedback
```

Na retirada, `store_state` é a pose segura de armazenamento e `retrieve_state`
é a pose baixa onde a garra alcança o objeto. Para o compartimento `left`, a
sequência completa é `pre_grip` → `deposit_cube_left` → `pick_cube_left` →
fechar em `grip` → `deposit_cube_left` → `home`.

Depósito em uma pose explícita do TCP (`base_link`):

```bash
ros2 action send_goal manipulation/place_at_pose interfaces/action/PlaceAtPose \
  "{release_pose: {header: {frame_id: base_link}, pose: \
    {position: {x: 0.20, y: 0.0, z: 0.10}, orientation: {w: 1.0}}}}" \
  --feedback
```

Interface para depósito automático em mesa:

```bash
ros2 action send_goal manipulation/place_on_table interfaces/action/PlaceOnTable \
  "{ws_height_cm: 12.5, analyze_apriltags: true, \
    analyze_containers: false}" --feedback
```

Interface para depósito em contêiner:

```bash
ros2 action send_goal manipulation/place_in_container \
  interfaces/action/PlaceInContainer \
  "{ws_height_cm: 12.5, container_color: 1}" --feedback
```

Empilhamento sobre o cubo cuja AprilTag é 5:

```bash
ros2 action send_goal manipulation/stack interfaces/action/StackObject \
  "{support_tag_id: 5}" --feedback
```

Depósito na prateleira fixa:

```bash
ros2 action send_goal manipulation/place_on_shelf interfaces/action/PlaceOnShelf \
  "{}" --feedback
```

O pacote não contém valores inventados para destinos ainda não medidos. O
único compartimento interno habilitado é `left`, pois é o único com poses
existentes no SRDF atual.
