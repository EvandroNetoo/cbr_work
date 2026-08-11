# cbr_bringup

`robot.launch.py` é o perfil embarcado do robô. Ele executa hardware,
`ros2_control`, controllers e `move_group` na Banana Pi, sem iniciar RViz,
Gazebo ou teleop. O argumento `enable_apriltag` permanece desligado por
default e só deve ser habilitado quando um driver de câmera já estiver
publicando os tópicos configurados.

A futura lógica autônoma deve ser incluída neste perfil depois que os
controllers e o `/move_action` estiverem prontos; o notebook não é requisito
para essa inicialização.
