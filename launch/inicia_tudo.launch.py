from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Caminhos dos launches
    pkg_1 = FindPackageShare('trabalho_1').find('trabalho_1')
    launch_1 = os.path.join(pkg_1, 'launch', 'inicia_simulacao.launch.py')
    launch_2 = os.path.join(pkg_1, 'launch', 'carrega_robo.launch.py')

    # Inclui os launches
    primeiro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_1)
    )

    segundo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_2)
    )

    # Roda o script com ros2 run (após alguns segundos, para garantir que os nós já subiram)
    rodar_script = TimerAction(
        period=5.0,  # espera 5 segundos
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'trabalho_1', 'controle_robo'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        primeiro_launch,
        segundo_launch,
        rodar_script
    ])
