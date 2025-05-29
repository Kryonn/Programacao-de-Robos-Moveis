#!/bin/bash

# Caminho até o workspace
WS=~/ros2_ws

# Compilar o workspace
cd $WS
colcon build --symlink-install --packages-select trabalho_1

# Atualizar o ambiente
source install/local_setup.bash

# Abrir o terminal do Gazebo
gnome-terminal -- bash -c "
cd $WS;
source install/local_setup.bash;
ros2 launch trabalho_1 inicia_simulacao.launch.py;
exec bash
"

# Abrir o terminal do robô
gnome-terminal -- bash -c "
cd $WS;
source install/local_setup.bash;
ros2 launch trabalho_1 carrega_robo.launch.py;
exec bash
"

# Abrir o terminal do controle
gnome-terminal -- bash -c "
cd $WS;
source install/local_setup.bash;
sleep 15;
ros2 run trabalho_1 controle_robo;
exec bash
"
