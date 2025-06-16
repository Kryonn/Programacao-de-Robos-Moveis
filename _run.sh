#!/bin/bash

# Caminho até o workspace
WS=~/ros2_ws

# Salva o diretório atual
DIR_INICIO=$(pwd)

# Compilar o workspace
cd "$WS" || exit
colcon build --symlink-install --packages-select trabalho_1

# Atualizar o ambiente
source /opt/ros/humble/setup.bash  # Ajuste para sua versão do ROS
source install/local_setup.bash

export LIBGL_ALWAYS_SOFTWARE=1

# Função para abrir terminais (compatível com WSL1 e WSL2)
open_terminal() {
    local title=$1
    local command=$2
    
    # Verifica se estamos no WSL2 com GUI habilitada
    if [ -n "$WSL_DISTRO_NAME" ] && [ -n "$DISPLAY" ]; then
        xterm -T "$title" -e "bash -c '$command; exec bash'" &
    else
        # Fallback para abrir no Windows Terminal se disponível
        if cmd.exe /c where wt >/dev/null 2>&1; then
            cmd.exe /c "wt -p \"Ubuntu\" --title \"$title\" bash -c \"$command; exec bash\""
        else
            # Último recurso - abre em janelas separadas do CMD
            cmd.exe /c start bash -c "$command"
        fi
    fi
}

# Abrir a simulação do Gazebo
cd $WS
source /opt/ros/humble/setup.bash
source install/local_setup.bash
cd $DIR_INICIO
pkill -f gazebo && pkill -f roslaunch && pkill -f rosmaster 
ros2 launch trabalho_1 inicia_tudo.launch.py