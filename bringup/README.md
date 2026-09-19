# bringup

O bringup oferece dois perfis distribuídos e mantém `robot.launch.py`
inalterado como rollback monolítico.

Na Banana Pi, `hardware.launch.py` inicia apenas drivers físicos e controle de
baixo nível: braço, base, LiDAR, IMU, VL53, um único `controller_manager`,
`joint_state_broadcaster` e os controllers. Na Raspberry Pi 4,
`processing.launch.py` inicia `robot_state_publisher`, EKF e, por padrão,
visão, localização/Nav2, MoveIt/manipulação e missão. RViz e teleop continuam
separados no perfil de workstation.

```bash
# Banana Pi
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" banana
ros2 launch bringup hardware.launch.py port:=/dev/ttyUSB0

# Raspberry Pi
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" rasp
ros2 launch bringup processing.launch.py
```

O mapa padrão é `arena`. A troca usa somente o nome instalado:

```bash
ros2 launch bringup processing.launch.py map:=arena_nova3
```

Os quatro grupos opcionais são independentes e ficam ativos por padrão:

```bash
ros2 launch bringup processing.launch.py \
  enable_vision:=false \
  enable_navigation:=false \
  enable_manipulation:=false \
  enable_mission:=false
```

`enable_navigation` controla conjuntamente map server, AMCL e Nav2. O EKF e o
`robot_state_publisher` permanecem sempre ativos no perfil de processamento.
Quando `enable_manipulation` está ativo, MoveIt e o servidor semântico só são
iniciados depois que os controllers remotos do braço respondem como ativos.

O perfil distribuído usa câmera a 15 FPS, detector AprilTag limitado a 10 Hz e
`controller_manager` a 30 Hz. O braço amostra o setpoint mais recente a 60 Hz,
mas só escreve na serial quando ele muda; a taxa maior evita alias com o loop
de controle. A base mantém somente o comando mais recente e reenvia o alvo
estacionário a 5 Hz. O LiDAR permanece ativo no perfil de hardware; MoveIt
pode ser desligado com `enable_manipulation:=false`.

As poses cartesianas e as detecções usadas pela manipulação são expressas em
`arm_base_link`. No perfil composto, `base_link` pertence ao chassi e difere da
base do braço pelo yaw físico de 90 graus do suporte.

O servidor `manipulation_server` é iniciado depois da ativação dos controllers,
junto ao MoveIt. O gerenciador de missão pode ser desligado independentemente;
o notebook não é requisito para a autonomia.

No notebook, a workstation consome os tópicos publicados pelo robô sem iniciar
drivers, controllers ou outro `robot_state_publisher`:

```bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" notebook
ros2 launch bringup workstation.launch.py
ros2 launch bringup workstation.launch.py enable_keyboard_teleop:=true
ros2 launch bringup workstation.launch.py enable_xbox_teleop:=true
```

O RViz inclui modelo, LiDAR, odometria, câmera, MotionPlanning e painéis
preparados para Nav2. Para diagnóstico, o perfil leve de navegação também pode
ser iniciado isoladamente depois do map server/AMCL e da pose inicial:

```bash
ros2 launch bringup navigation.launch.py
```

O launch encontra o YAML e a árvore BT no `share/bringup`; nenhum caminho do
workspace fica gravado nos parâmetros. Para diagnóstico ainda é possível
sobrescrever `params_file`, mas isso não é necessário no uso normal. O launch
legado `telemetry.launch.py` continua disponível para abrir apenas o mesmo
painel RViz.

No Xbox, `RB` habilita o movimento, `LB` ativa turbo, o stick esquerdo comanda
X/Y, o stick direito comanda yaw e `B` publica parada e solicita cancelamento
dos goals Nav2. Soltar `RB` ou perder a conexão também publica parada.

Os drivers físicos podem levar cerca de 20 segundos para conectar na Banana
Pi. O launch aguarda até 45 segundos pelos estados completos das seis juntas
comandadas do braço, das quatro rodas e da IMU antes de iniciar os controllers.
A saída de qualquer driver encerra o perfil de hardware.

```bash
ros2 launch bringup hardware.launch.py \
  port:=/dev/ttyUSB0 \
  hardware_state_timeout:=60.0
```

Para liberar as juntas do braço para posicionamento manual a qualquer momento,
mantendo os estados publicados pelo robô composto:

```bash
ros2 service call /so101_hardware_node/set_torque std_srvs/srv/SetBool \
  "{data: false}"
```

Nesse modo os comandos do braço são ignorados. Sustente-o antes da chamada,
pois as juntas podem cair sob o próprio peso. Use `"{data: true}"` no mesmo
serviço para reativar o torque sem reiniciar o robô.

Para alterar as taxas de câmera e controle no perfil distribuído:

```bash
ros2 launch bringup hardware.launch.py \
  port:=/dev/ttyUSB0 \
  controller_update_rate:=50 \
  base_deduplicate_commands:=false

ros2 launch bringup processing.launch.py camera_framerate:=30.0
```

## Validação distribuída

Meça por 60 segundos em repouso e durante uso, ocultando threads no `htop`
(`H`) para comparar processos agregados em cada placa:

```bash
# Raspberry Pi
pidstat -durwt -p ALL 1 60
ros2 topic hz /camera/image_rect
ros2 topic hz /apriltags/detections_camera

# Banana Pi
pidstat -durwt -p ALL 1 60
ros2 topic hz /so101_hardware/raw_joint_states
ros2 topic hz /base_hardware/raw_joint_states
```

Os valores esperados são 14–16 Hz para a imagem, 9–11 Hz para o detector e
28–32 Hz para os dois estados de hardware. Compare os dois perfis sob a mesma
temperatura/frequência de CPU. A meta é reduzir pelo menos 25% da CPU agregada
com visão e 30% em repouso, sem falhas seriais ou regressão de trajetória/parada.

Os argumentos de hardware pertencem ao braço e ao gate geral. A base não possui
configuração por CLI: hardware, geometria e controllers ficam nos YAMLs dos
pacotes `base_hardware`, `base_bringup` e `bringup`.

## Rede e Cyclone DDS

Os perfis `config/cyclonedds_{banana,rasp,notebook,wsl}.xml` foram escritos para
Cyclone DDS 0.10.5 / ROS 2 Jazzy. O pacote declara `rmw_cyclonedds_cpp` como
dependência. Depois de instalar as dependências nas três máquinas:

```bash
colcon build --symlink-install --packages-select bringup
source install/setup.bash
```

Use `source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash"`
com o argumento da máquina em **cada terminal**, incluindo CLI, RViz e teleop.
No checkout, também é possível usar
`source src/cbr_work/bringup/scripts/dds_environment.bash banana` (ou `rasp`,
`notebook`, `wsl`). Serviços systemd precisam receber o mesmo ambiente antes de
executar o launch; um `source` no terminal não altera serviços já iniciados.

| Máquina | Ethernet | Wi-Fi |
|---|---|---|
| Banana | eth0, 10.50.0.1/30 | wlan1, DHCP |
| Raspberry | eth0, 10.50.0.2/30 | wlan0, DHCP |
| Notebook | — | wlp0s20f3, DHCP |

A configuração do Linux é um pré-requisito: Ethernet sem gateway/DNS e sem
rota padrão; Wi-Fi com gateway/DNS do hotspot. A rede do hotspot não pode
sobrepor a rede Ethernet. Os arquivos deste pacote não alteram endereços,
rotas, firewall, NetworkManager ou serviços do sistema.

No notebook, `DontRoute=true` descarta os endereços DDS fora da sub-rede
Wi-Fi, incluindo os `10.50.0.x` anunciados pelas placas. Sem isso, é possível
descobrir tópicos mas não receber os dados. Esse ajuste pressupõe os três
Wi-Fi na mesma sub-rede; precisa ser revisto se houver VPN ou roteamento
entre sub-redes. Ele não foi aplicado aos perfis das placas.

O script seleciona RMW e XML, define domínio 10 e descoberta SUBNET, e remove
`ROS_LOCALHOST_ONLY` e `ROS_STATIC_PEERS` da sessão para evitar conflito com o
perfil. Ele não para processos nem o daemon. Ao migrar, pare os launches antigos,
configure o ambiente, execute `ros2 daemon stop` e inicie novamente. Aplicar o
ambiente a um terminal não migra os nós que já estão executando.

Ethernet tem prioridade 100 e Wi-Fi 10. Dados usam unicast; multicast fica
restrito à descoberta SPDP. A descoberta é recebida nas interfaces configuradas,
e o peer fixo da outra placa e localhost ajudam na descoberta remota e local.
Há até 65 índices automáticos (0–64); a faixa não deve crescer sem medição,
pois aumenta as sondagens. O XML do notebook exige seu Wi-Fi. Nas placas o
Wi-Fi é opcional na inicialização, para permitir operação sem hotspot.

### Workstation no WSL 2

O perfil `notebook` exige a interface `wlp0s20f3` do Ubuntu original. No WSL,
após reconstruir o pacote `bringup`, use:

```bash
source install/setup.bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" wsl
ros2 launch bringup workstation.launch.py
```

O perfil `wsl` escolhe automaticamente uma interface disponível, geralmente
`eth0` no NAT ou `eth1` no modo espelhado. Isso resolve a falha de
inicialização causada pelo nome antigo. Um IP como `172.30.x.x` em `eth0`
costuma indicar NAT do WSL 2: o RViz pode abrir, mas a descoberta multicast e
os dados DDS das placas podem não atravessar essa
rede. Para comunicação com o robô, no Windows 11 22H2 ou superior, configure
`networkingMode=mirrored` em `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Execute `wsl --shutdown` no PowerShell e abra o WSL novamente. Confira
`ip -br addr` e teste `ros2 topic list --no-daemon --spin-time 5` com as placas
ligadas no mesmo hotspot. No modo espelhado, o daemon do `ros2cli` pode encerrar
após ficar inativo e a conexão local seguinte pode expirar. Use `--no-daemon`
para uma consulta confiável. Se quiser usar `ros2 topic list` sem essa opção,
execute `ros2 daemon start` antes; talvez seja necessário repetir isso depois
de um período sem comandos CLI.
O perfil `wsl` descobre as placas por multicast, sem IPs fixos no XML.
Se o multicast chega às placas mas não entra no WSL, configure no PowerShell
como administrador uma regra Hyper-V para as portas UDP de DDS no domínio 10
(9900–10039):

```powershell
New-NetFirewallHyperVRule `
  -Name 'ROS2-WSL-Domain10' `
  -DisplayName 'ROS 2 WSL Domain 10' `
  -Direction Inbound -Action Allow `
  -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
  -Protocol UDP `
  -LocalPorts '9900-10039' `
  -RemoteAddresses '192.168.1.216','192.168.1.114'
```

Atualize também a regra se os IPs das placas mudarem. Se ainda não houver
tópicos, verifique se o hotspot permite multicast entre clientes. O modo
espelhado depende da versão do Windows; no Windows 10, usar um Ubuntu nativo
na rede do robô é o caminho mais simples para a comunicação DDS.

Isso implementa **preferência Ethernet**, não isolamento: SPDP continua no
Wi-Fi, e prioridade não proíbe outros caminhos. O hotspot precisa permitir
comunicação entre clientes e multicast. Se bloquear apenas multicast, será
necessário configurar descoberta unicast para os endereços Wi-Fi atuais.
Inicie com os IPs já atribuídos. Mudança de IP/hotspot ou interface que aparece
após a inicialização pode exigir reiniciar os processos; não há promessa de
reconexão transparente. Retirar o cabo não deve ser usado como teste de
continuidade garantida: uma política estrita sem Wi-Fi não teria caminho reserva.

Os limites de payload UDP são 1400 bytes (incluindo RTPS), e os fragmentos DDS
são 1200 bytes. São valores iniciais para IPv4 com MTU de pelo menos 1500 no
caminho inteiro. Mensagens ROS maiores continuam permitidas. Isso aumenta a
quantidade de pacotes e deve ser medido com imagens e mapas. QoS dos tópicos,
serviços e actions não foi alterado; assinantes reliable lentos no notebook
podem causar retransmissões/pressão de histórico mesmo com o enlace Ethernet.

### Aceitação nas máquinas

Verifique primeiro versões, ambiente, interfaces e rotas:

```bash
dpkg-query -W ros-jazzy-cyclonedds ros-jazzy-rmw-cyclonedds-cpp
printenv RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_DOMAIN_ID
ip -br address
ip route
# Banana; no Raspberry use 10.50.0.1:
ip route get 10.50.0.2
```

O destino da outra placa deve indicar `eth0`; o IP do notebook deve indicar a
interface Wi-Fi da placa. Para observar tráfego real, capture simultaneamente
Ethernet e Wi-Fi durante o uso (duas sessões, interromper com Ctrl-C):

```bash
sudo tcpdump -ni eth0 -s 0 -w /tmp/cbr-eth.pcap udp
# Banana; no Raspberry substituir wlan1 por wlan0:
sudo tcpdump -ni wlan1 -s 0 -w /tmp/cbr-wifi.pcap udp
```

Analise RTPS no Wireshark por IP de origem/destino e tipo de submensagem.
Pacotes de descoberta Wi-Fi são esperados; diferencie-os dos dados. Contagem
de bytes da interface e `ros2 topic list` sozinhos não comprovam o caminho.
`ros2 topic hz/bw` também criam assinaturas e podem mudar a carga observada.

Valide com o robô parado antes de comandar movimento:

1. Placas sem notebook: estados, sensores, TF e ativação de controllers/Nav2.
2. Notebook conectado depois: `/tf_static`, mapa e `/mission/state` existentes.
3. Notebook assinando o mesmo tópico da outra placa: dados entre placas no cabo.
4. Actions: descoberta, feedback, resultado e cancelamento; serviços também.
5. Imagens/mapas: frequência, latência, perdas, fragmentação IP e CPU.
6. Desconexão/reconexão do notebook e do Wi-Fi: ausência de regressão no cabo.
7. Reinício de cada placa, diferentes ordens de inicialização e troca de hotspot.

### Retorno ao Fast DDS

Pare os processos Cyclone. Em cada terminal das três máquinas:

```bash
source "$(ros2 pkg prefix --share bringup)/scripts/dds_environment.bash" fastdds
ros2 daemon stop
```

Reinicie os launches habituais. O perfil `fastdds_nav2.xml` e sua aplicação pelo
launch de navegação foram preservados; eles são ignorados pelo Cyclone e voltam
a ser usados com Fast DDS. A opção `fastdds` mantém domínio 10 e descoberta
SUBNET e remove `CYCLONEDDS_URI`; não restaura outras personalizações antigas
do ambiente. Não é necessário remover o pacote Cyclone.
