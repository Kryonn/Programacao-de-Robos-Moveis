from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # ------------------------------------------------------
    # Argumento para o nome do mundo (arquivo SDF)
    # ------------------------------------------------------
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.sdf',
        description='Nome do arquivo .sdf do mundo a ser carregado'
    )

    world_file = LaunchConfiguration('world')

    # ------------------------------------------------------
    # Caminhos de pacotes e arquivos
    # ------------------------------------------------------
    pkg_share = FindPackageShare("prm").find("prm")

    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file
    ])

    gz_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models")
    ])

    # ------------------------------------------------------
    # Configuração de ambiente
    # ------------------------------------------------------
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ])
    }

    set_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gz_models_path
    )

    # ------------------------------------------------------
    # Lançamento do Gazebo com GUI
    # ------------------------------------------------------
    gz_verbosity = '3'

    gazebo_gui = ExecuteProcess(
        cmd=[
            FindExecutable(name="ign"), 'gazebo',
            '-v', gz_verbosity,
            world_path
        ],
        output='screen',
        additional_env=gz_env,
        shell=False
    )

    # ------------------------------------------------------
    # LaunchDescription final
    # ------------------------------------------------------
    return LaunchDescription([
        declare_world_arg,
        set_resource_path,
        gazebo_gui
    ])
