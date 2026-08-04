# so_arm_101_description

Pacote responsável pela descrição do follower: URDF/Xacro, `ros2_control`,
meshes e limites cinemáticos. Não contém launch de hardware, MoveIt ou
teleoperação.

O arquivo principal é `urdf/so_101.urdf.xacro`. Os limites em
`config/joint_limits.yaml` são consumidos pelo Xacro e também são a fonte dos
limites usados pelo teleop e pelo adapter Python.
