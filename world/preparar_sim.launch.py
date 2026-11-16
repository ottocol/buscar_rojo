"""
preparar_sim.launch.py  -  Simulador mvsim + Nav2 + pose inicial, con un solo comando.

USO (no hace falta compilar ni hacer source del workspace, solo el de ROS 2):

    cd <ruta>/buscar_rojo/world
    ros2 launch preparar_sim.launch.py

    # con la pose inicial explícita (yaw en grados, como en el .world.xml de mvsim):
    ros2 launch preparar_sim.launch.py x:=1.5 y:=-0.5 yaw_deg:=90

    # desde cualquier sitio, dando la ruta al fichero:
    ros2 launch ~/ros2_ws/src/buscar_rojo/world/preparar_sim.launch.py

ARGUMENTOS
    world_dir   carpeta con buscar_rojo.world.xml, mapa.yaml y nav2.yaml
                (por defecto: la carpeta donde está este launch, o ../world, o la carpeta actual)
    x, y        posición inicial en el mapa (m)            por defecto: -10, 10
    yaw_deg     orientación inicial (grados)               por defecto: -90
                Los valores por defecto son los del <init_pose> de buscar_rojo.world.xml.
                Si cambias la pose en el mundo, cámbiala también aquí (POSE_INICIAL).
    nav2_delay  segundos de espera antes de lanzar Nav2 (por defecto 3)

CÓMO SE FIJA LA POSE INICIAL
    En lugar de publicar en /initialpose (el nodo pose_inicial), se activan los parámetros
    set_initial_pose / initial_pose de AMCL en una copia temporal de nav2.yaml.
    AMCL aplica la pose él solo al activarse: no hay que esperar a que esté listo
    ni arriesgarse a publicar antes de que exista el suscriptor.
"""

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLD_FILE = 'buscar_rojo.world.xml'
MAP_FILE = 'mapa.yaml'
NAV2_PARAMS = 'nav2.yaml'

# Debe coincidir con <init_pose> del robot en buscar_rojo.world.xml (x, y, yaw en grados)
POSE_INICIAL = (-10.0, 10.0, -90.0)

THIS_DIR = os.path.dirname(os.path.realpath(__file__))


def default_world_dir():
    """Carpeta del launch, ../world o la carpeta actual: la primera que tenga el mundo."""
    for d in (THIS_DIR, os.path.join(THIS_DIR, '..', 'world'), os.getcwd()):
        if os.path.isfile(os.path.join(d, WORLD_FILE)):
            return os.path.abspath(d)
    return os.getcwd()


def nav2_params_with_initial_pose(params_file, x, y, yaw_deg):
    """Copia temporal de nav2.yaml con la pose inicial puesta en AMCL."""
    with open(params_file) as f:
        params = yaml.safe_load(f) or {}
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': math.radians(yaw_deg)}
    tmp = tempfile.NamedTemporaryFile('w', prefix='nav2_buscar_rojo_', suffix='.yaml',
                                      delete=False)
    yaml.safe_dump(params, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    world_dir = os.path.abspath(
        os.path.expanduser(LaunchConfiguration('world_dir').perform(context)))
    world_file = os.path.join(world_dir, WORLD_FILE)
    map_file = os.path.join(world_dir, MAP_FILE)
    params_file = os.path.join(world_dir, NAV2_PARAMS)

    missing = [p for p in (world_file, map_file, params_file) if not os.path.isfile(p)]
    if missing:
        raise RuntimeError(
            'No encuentro: ' + ', '.join(missing) +
            '\nLanza desde la carpeta world del paquete o pasa world_dir:=<carpeta>')

    x, y, yaw_deg = (float(LaunchConfiguration(n).perform(context))
                     for n in ('x', 'y', 'yaw_deg'))

    nav2_params = nav2_params_with_initial_pose(params_file, x, y, yaw_deg)
    delay = float(LaunchConfiguration('nav2_delay').perform(context))

    mvsim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mvsim'), 'launch', 'launch_world.launch.py')),
        launch_arguments={
            'world_file': world_file,
            'do_fake_localization': 'false',
        }.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
        }.items())

    return [
        LogInfo(msg=f'[preparar_sim] Mundo: {world_dir}'),
        LogInfo(msg=f'[preparar_sim] Pose inicial: '
                    f'x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f} grados'),
        LogInfo(msg=f'[preparar_sim] Parámetros de Nav2 usados: {nav2_params}'),
        mvsim,
        TimerAction(period=delay, actions=[nav2]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_dir', default_value=default_world_dir(),
                              description='Carpeta con el mundo, el mapa y nav2.yaml'),
        DeclareLaunchArgument('x', default_value=str(POSE_INICIAL[0]),
                              description='x inicial (m)'),
        DeclareLaunchArgument('y', default_value=str(POSE_INICIAL[1]),
                              description='y inicial (m)'),
        DeclareLaunchArgument('yaw_deg', default_value=str(POSE_INICIAL[2]),
                              description='Orientación inicial (grados)'),
        DeclareLaunchArgument('nav2_delay', default_value='3.0',
                              description='Segundos de espera antes de lanzar Nav2'),
        OpaqueFunction(function=launch_setup),
    ])
