#!/usr/bin/env python3
"""
Nodo deambulador: mueve el robot al azar evitando obstáculos, pero SOLO
mientras alguien lo tenga activado.

    /scan (LaserScan) ──▶ [deambulador] ──▶ /cmd_vel (Twist)
                               ▲
    /deambulador/activar (std_srvs/SetBool)   ← el coordinador lo enciende/apaga

Es un nodo de *comportamiento* (actúa sobre el robot). La idea clave es que
el bucle de control (10 Hz) vive aquí, no en el coordinador: el coordinador
(FSM o BT) solo decide CUÁNDO hay que deambular, no CÓMO.

¿Por qué hay que poder desactivarlo? Porque Nav2 también publica en /cmd_vel.
Si dos nodos publican a la vez en /cmd_vel el robot recibe órdenes
contradictorias y hace cosas raras. Por eso, cuando está desactivado, este
nodo no publica nada.

Probarlo a mano:
    ros2 run buscar_rojo deambulador
    ros2 service call /deambulador/activar std_srvs/srv/SetBool "{data: true}"
    ros2 service call /deambulador/activar std_srvs/srv/SetBool "{data: false}"
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from buscar_rojo.comun import SRV_DEAMBULAR, TOPIC_LASER, TOPIC_VEL


class Deambulador(Node):

    def __init__(self):
        super().__init__('deambulador')

        # Parámetros de la conducta (valores tentativos: ajustadlos)
        self.v_lineal = self.declare_parameter('vel_lineal', 0.4).value        # m/s
        self.v_angular = self.declare_parameter('vel_angular', 0.8).value      # rad/s
        self.dist_seguridad = self.declare_parameter('dist_seguridad', 1.0).value  # m

        self.activo = False
        self.ultimo_scan = None
        # Giro aleatorio "de crucero" que se cambia cada pocos segundos para
        # que el robot no vaya siempre en línea recta (eso es lo "al azar")
        self.giro_crucero = 0.0
        self.ticks_hasta_cambio = 0

        self.pub_vel = self.create_publisher(Twist, TOPIC_VEL, 10)
        self.create_subscription(LaserScan, TOPIC_LASER, self.laser_cb, qos_profile_sensor_data)
        self.create_service(SetBool, SRV_DEAMBULAR, self.activar_cb)
        self.create_timer(0.1, self.control)   # bucle de control a 10 Hz

        self.get_logger().info(f'Listo. Actívame con el servicio {SRV_DEAMBULAR}')

    # ------------------------------------------------------------------
    # Servicio: activar / desactivar
    # ------------------------------------------------------------------
    def activar_cb(self, peticion: SetBool.Request, respuesta: SetBool.Response):
        self.activo = peticion.data
        if not self.activo:
            self.pub_vel.publish(Twist())   # parar el robot al desactivar
        respuesta.success = True
        respuesta.message = 'deambulando' if self.activo else 'parado'
        self.get_logger().info(f'Deambulador {respuesta.message}')
        return respuesta

    def laser_cb(self, msg: LaserScan):
        self.ultimo_scan = msg

    # ------------------------------------------------------------------
    # Bucle de control
    # ------------------------------------------------------------------
    def control(self):
        if not self.activo or self.ultimo_scan is None:
            return

        scan = self.ultimo_scan
        # Distancia mínima en tres sectores (ángulos en grados, 0 = delante,
        # positivos a la izquierda)
        frente = self.distancia_minima(scan, -15, 15)
        izq = self.distancia_minima(scan, 15, 60)
        der = self.distancia_minima(scan, -60, -15)

        cmd = Twist()
        if min(frente, izq, der) > self.dist_seguridad:
            # Vía libre: avanzar con un giro suave aleatorio
            self.ticks_hasta_cambio -= 1
            if self.ticks_hasta_cambio <= 0:
                self.giro_crucero = random.uniform(-0.3, 0.3) * self.v_angular
                self.ticks_hasta_cambio = random.randint(20, 40)   # 2-4 s
            cmd.linear.x = self.v_lineal
            cmd.angular.z = self.giro_crucero
        else:
            # Obstáculo cerca: casi parado, girar hacia el lado más despejado
            cmd.linear.x = 0.05 if frente > 0.5 * self.dist_seguridad else 0.0
            cmd.angular.z = self.v_angular if izq > der else -self.v_angular
        self.pub_vel.publish(cmd)

    @staticmethod
    def distancia_minima(scan: LaserScan, desde_grados: float, hasta_grados: float) -> float:
        """Distancia mínima válida del láser entre dos ángulos (en grados)."""
        i0 = int((math.radians(desde_grados) - scan.angle_min) / scan.angle_increment)
        i1 = int((math.radians(hasta_grados) - scan.angle_min) / scan.angle_increment)
        i0, i1 = max(0, i0), min(len(scan.ranges) - 1, i1)
        validas = [r for r in scan.ranges[i0:i1 + 1]
                   if math.isfinite(r) and scan.range_min < r < scan.range_max]
        # Si no hay lecturas válidas suponemos que no hay nada (rayos al infinito)
        return min(validas) if validas else scan.range_max


def main():
    rclpy.init()
    nodo = Deambulador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
