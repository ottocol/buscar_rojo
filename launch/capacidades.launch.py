"""
Lanza los tres nodos "capacidad": detector_rojo, deambulador y fotografo.

    ros2 launch buscar_rojo capacidades.launch.py

Corren en paralelo, cada uno en su propio proceso, y se quedan esperando.
Después se lanza UN coordinador (fsm_yasmin o bt_pytrees) en otro terminal.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='buscar_rojo', executable='detector_rojo', output='screen',
             parameters=[{'umbral_pixeles': 300, 'frames_confirmacion': 3}]),
        Node(package='buscar_rojo', executable='deambulador', output='screen',
             parameters=[{'vel_lineal': 0.4, 'vel_angular': 0.8, 'dist_seguridad': 1.0}]),
        Node(package='buscar_rojo', executable='fotografo', output='screen',
             parameters=[{'carpeta': '~/fotos_robot'}]),
    ])
