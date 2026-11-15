#!/usr/bin/env python3
"""
Ejemplo mínimo: navegar a un punto del mapa por código con Nav2.

    ros2 run buscar_rojo ir_a_punto --ros-args -p x:=-10.0 -p y:=-9.0

Es lo mismo que hacer "Nav2 Goal" en rviz2: por debajo se envía un goal a la
acción /navigate_to_pose (tipo nav2_msgs/action/NavigateToPose). Aquí usamos
nav2_simple_commander, que lo esconde tras goToPose() / isTaskComplete().

Requisito: Nav2 en marcha y el robot localizado (pose_inicial o rviz2).
"""
import math

import rclpy
from rclpy.signals import SignalHandlerOptions
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from buscar_rojo.comun import crear_pose


def main():
    # Sin el manejador de Ctrl-C de rclpy, para poder cancelar la navegación al salir
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator(node_name='ir_a_punto')
    x = nav.declare_parameter('x', -10.0).value
    y = nav.declare_parameter('y', -9.0).value
    yaw = math.radians(nav.declare_parameter('yaw_grados', 0.0).value)

    destino = crear_pose(x, y, yaw)
    destino.header.stamp = nav.get_clock().now().to_msg()
    nav.goToPose(destino)            # envía el goal y vuelve enseguida (no bloquea)

    try:
        # Mientras navega podemos hacer otras cosas: aquí solo mostramos el feedback
        i = 0
        while not nav.isTaskComplete():  # isTaskComplete() también procesa los callbacks
            i += 1
            feedback = nav.getFeedback()
            if feedback and i % 10 == 0:
                nav.get_logger().info(f'Faltan {feedback.distance_remaining:.2f} m')
    except KeyboardInterrupt:
        nav.cancelTask()                 # Ctrl-C: que el robot no siga navegando solo
        while not nav.isTaskComplete():  # esperar a que Nav2 confirme la cancelación
            pass

    resultado = nav.getResult()
    if resultado == TaskResult.SUCCEEDED:
        nav.get_logger().info('¡Destino alcanzado!')
    elif resultado == TaskResult.CANCELED:
        nav.get_logger().warn('Navegación cancelada')
    else:
        nav.get_logger().error('Navegación fallida (¿destino inalcanzable u ocupado?)')

    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
