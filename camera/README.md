# CBR Camera

Este pacote isola a câmera do braço, do MoveIt e do detector AprilTag. A visão
deve ser montada e validada nesta ordem:

```text
dispositivo V4L2 -> imagem bruta ROS -> calibração intrínseca
                 -> imagem retificada -> TF de montagem -> AprilTag
```

Cada seta é uma etapa separada. Não use o AprilTag para testar se a câmera
abre e não invente uma transformação TF antes de medir a montagem física.

## Conceitos

- `/dev/videoN`: interface Linux da câmera. Nem toda interface `videoN`
  fornece imagens.
- `/camera/image_raw`: pixels capturados, ainda com distorção da lente.
- `/camera/camera_info`: modelo óptico da câmera. Antes da calibração, a
  matriz `k` pode estar zerada.
- `/camera/image_rect`: imagem sem a distorção modelada pela calibração.
- `camera_optical_frame`: eixos ópticos ROS: X para a direita, Y para baixo e
  Z para a frente.
- TF extrínseco: posição e orientação físicas da câmera no robô. Não faz parte
  da calibração do tabuleiro.

## Etapa 0: dependências

```bash
sudo apt install v4l-utils ros-jazzy-usb-cam \
  ros-jazzy-camera-calibration ros-jazzy-image-proc \
  ros-jazzy-rqt-image-view
```

## Etapa 1: selecionar o dispositivo

Liste as interfaces e os formatos que realmente capturam vídeo:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video1 --list-formats-ext
```

Neste hardware, `/dev/video1` foi validado com MJPG em 320 x 240 a 30 Hz. O
perfil embarcado usa 15 Hz para reduzir decode, retificação e cópias de memória:

```bash
v4l2-ctl -d /dev/video1 \
  --set-fmt-video=width=320,height=240,pixelformat=MJPG \
  --set-parm=30 --stream-mmap=3 \
  --stream-count=100 --stream-to=/dev/null
```

Use `video1` no comando somente se ele listar formatos de captura e concluir
esse teste. Se outro programa estiver usando a webcam, descubra-o com:

```bash
fuser -v /dev/video0 /dev/video1
```

Depois registre o dispositivo, a resolução e o formato escolhidos em
`config/camera.yaml`. O YAML define o default de 15 Hz e o argumento explícito
`framerate` permite teste A/B sem criar outro arquivo de configuração.

## Etapa 2: publicar apenas a imagem bruta

Na raiz do workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera
source install/setup.bash
ros2 launch camera camera.launch.py framerate:=15.0
```

Se a câmera oferecer somente YUYV, use `pixel_format: yuyv` no YAML. Use
exatamente um formato e uma resolução mostrados pelo `v4l2-ctl`.

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic hz /camera/image_raw
rqt_image_view /camera/image_raw
```

Critério para avançar: imagem estável, sem erros do driver e frequência
próxima da configurada. O aviso de calibração ausente ainda é esperado.

### Controles desta câmera

O kernel usa nomes V4L2 novos que o `usb_cam` 0.8.1 não reconhece ao tentar
configurar alguns controles pelos nomes antigos. Depois de iniciar o driver,
aplique os controles reais do dispositivo:

```bash
v4l2-ctl -d /dev/video1 --set-ctrl=brightness=0
v4l2-ctl -d /dev/video1 --set-ctrl=power_line_frequency=2
v4l2-ctl -d /dev/video1 --set-ctrl=white_balance_automatic=1
v4l2-ctl -d /dev/video1 --set-ctrl=auto_exposure=3
```

`power_line_frequency=2` seleciona 60 Hz. Os avisos sobre
`white_balance_temperature_auto`, `exposure_auto` e `focus_auto` são causados
pela diferença de nomes, não por uma falha de captura.

## Etapa 3: calibração intrínseca

A calibração atualmente versionada em `config/camera_info.yaml` foi obtida
em 320 x 240, foco fixo 30, com um tabuleiro de 9 x 6 cantos internos e
quadrados medidos com 24 mm. Foram aceitas 65 amostras. Para reproduzi-la:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.024 --no-service-check \
  --ros-args \
  --remap image:=/camera/image_raw \
  --remap camera:=/camera
```

### Alvo A4 alternativo

Mantenha a resolução definitiva. O arquivo
`calibration/checkerboard_8x6_25mm_a4.svg` contém um alvo A4 paisagem com
9 x 7 quadrados, portanto 8 x 6 cantos internos, e quadrados de 25 mm.
Imprima em escala 100%, sem "ajustar à página", e confirme com uma régua que
a linha de controle mede exatamente 100 mm. Cole a folha em uma superfície
plana e rígida. Para esse alvo, execute:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args \
  --remap image:=/camera/image_raw \
  --remap camera:=/camera
```

Substitua `0.025` pela medida real do lado do quadrado, em metros. Colete
imagens no centro, bordas, perto, longe e com inclinações. Clique em
`CALIBRATE` e depois em `SAVE`. Como este driver não anunciou o serviço
`set_camera_info`, a calibração foi executada com `--no-service-check`. O
arquivo `ost.yaml` salvo foi normalizado para o nome `camera` e versionado em:

```text
camera/config/camera_info.yaml
```

Reinicie o driver e confirme que `k[0]` e `k[4]` não são zero:

```bash
ros2 topic echo /camera/camera_info --once
```

## Etapa 4: retificação

Somente depois da calibração:

```bash
ros2 launch camera camera.launch.py \
  rectify:=true framerate:=15.0
```

Visualize `/camera/image_rect`. Linhas retas próximas às bordas devem parecer
mais retas que em `/camera/image_raw`.

## Etapa 5: montagem no robô

A transformação entre um elo do braço e `camera_optical_frame` será criada
somente depois de definir onde a câmera será presa e medir sua pose. Esta
versão do pacote não publica esse TF propositalmente.

## Etapa 6: AprilTag

Depois de validar separadamente imagem retificada, `CameraInfo` e TF, o perfil
`bringup robot.launch.py` passou a iniciar câmera, retificação e detector
AprilTag sempre.
