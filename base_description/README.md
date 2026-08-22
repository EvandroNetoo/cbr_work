# base_description

Modelo paramétrico inicial da base mecanum. O chassi acompanha a construção
física observada: uma armação inferior oca, composta por quatro barras inferiores,
quatro barras superiores e quatro colunas, sustenta uma plataforma superior
maciça e maior.

As dimensões atuais são apenas estimativas e estão concentradas no início do
Xacro. Os grupos `lower_frame_*`, `lower_beam_width` e `upper_platform_*`
controlam separadamente os dois níveis. Substitua também raio das rodas,
bitola, entre-eixos, massas, inércias e poses dos sensores pelas medições antes
de usar hardware real.

O único LiDAR é `lidar_front_link`, mantido com esse nome por compatibilidade
com o driver. Suas medidas e posição estão no grupo `lidar_*`; a propriedade
`lidar_center_height_above_ground` é medida verticalmente do chão até o centro
do cilindro laranja. O corpo quadrado preto fica acima dele, preso à base, e o
frame possui uma rotação de 180 graus por causa da montagem invertida.

A IMU também é paramétrica. O grupo `imu_*` controla suas dimensões, massa,
posição e orientação. `imu_x`, `imu_y` e `imu_z` são relativos ao `base_link`;
`imu_roll`, `imu_pitch` e `imu_yaw` são informados em radianos.

Visualização:

```bash
ros2 launch base_description display.launch.py
```

O launch carrega `rviz/base.rviz`, já configurado com frame fixo
`base_footprint`, `RobotModel`, TF e grade. O display do LiDAR também está
preparado, mas desligado durante a visualização puramente estrutural.

Sem os controles gráficos das juntas:

```bash
ros2 launch base_description display.launch.py use_gui:=false
```
