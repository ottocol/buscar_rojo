#!/usr/bin/env python3
"""
Da a AMCL la pose inicial del robot por código, en lugar de con "2D Pose
Estimate" en rviz2, y espera a que Nav2 esté listo para navegar.

    ros2 run buscar_rojo pose_inicial                    # (-10, 10, -90º), la pose inicial en buscar_rojo.world.xml
    ros2 run buscar_rojo pose_inicial --ros-args -p x:=0.0 -p y:=0.0 -p yaw_grados:=0.0

Usa nav2_simple_commander (paquete ros-jazzy-nav2-simple-commander), que
envuelve las acciones y servicios de Nav2 en llamadas Python sencillas.
"""
import math

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

from buscar_rojo.comun import crear_pose


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='pose_inicial')
    x = nav.declare_parameter('x', -10.0).value
    y = nav.declare_parameter('y', 10.0).value
    yaw = math.radians(nav.declare_parameter('yaw_grados', -90.0).value)

    pose = crear_pose(x, y, yaw)
    pose.header.stamp = nav.get_clock().now().to_msg()
    try:
        nav.setInitialPose(pose)       # publica en /initialpose
        nav.waitUntilNav2Active()      # espera a amcl y bt_navigator (y a que AMCL acepte la pose)
        nav.get_logger().info('Pose inicial fijada. Nav2 listo para navegar.')
    except KeyboardInterrupt:
        pass

    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
