Ambiente ROS2 Humble e Ubuntu 22.04

- Lançar o ambiente: ros2 launch robot_gazebo start_world_box_room.launch.py
- Lançar o robot: ros2 launch robot_gazebo spawn_robot_description_ros2_final.launch.xml
- Controle via o joystick: ros2 launch robot_description joy_command.launch.py
- Slam: ros2 launch slam online_async_launch.py, lembrar de mudar de navegação pra localização e vice-versa e o local do mapa
- Navegação: ros2 launch slam navigation2_launch.py, mostrar como usar o nav2 painel para waipoints
- Navegação com região de segurança: ros2 launch slam navigation2_filtered.launch.py, m
- Go to poses gerado de forma randômica: ros2 launch nav2_features go_to_poses.launch.py
