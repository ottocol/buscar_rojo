#!/usr/bin/env python3
"""
Nodo detector_rojo: dice continuamente si hay algo rojo en la imagen.

    /camera1_image (Image) ──▶ [detector_rojo] ──▶ /detector_rojo/detectado (Bool)
                                              └──▶ /detector_rojo/mascara   (Image)

Es un nodo de *percepción*: no sabe nada de la tarea, ni de máquinas de
estados, ni de si el robot se mueve o no. Solo convierte imágenes en un
dato sencillo (True/False). Por eso se puede probar por separado:

    ros2 run buscar_rojo detector_rojo
    ros2 topic echo /detector_rojo/detectado
    ros2 run rqt_image_view rqt_image_view /detector_rojo/mascara

y reutilizar en otra tarea distinta sin tocar una línea.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from buscar_rojo.comun import TOPIC_CAMARA, TOPIC_ROJO_DETECTADO, TOPIC_ROJO_MASCARA

# En HSV el rojo está en los dos extremos del círculo de tono (H en [0, 180)
# en OpenCV), así que hacen falta DOS rangos y luego juntar las máscaras.
ROJO_BAJO_1 = np.array([0, 120, 70])
ROJO_ALTO_1 = np.array([10, 255, 255])
ROJO_BAJO_2 = np.array([170, 120, 70])
ROJO_ALTO_2 = np.array([180, 255, 255])


class DetectorRojo(Node):

    def __init__(self):
        super().__init__('detector_rojo')

        # Parámetros: se pueden cambiar sin tocar el código, p.ej.
        #   ros2 run buscar_rojo detector_rojo --ros-args -p umbral_pixeles:=1000
        self.umbral = self.declare_parameter('umbral_pixeles', 300).value
        # Nº de imágenes seguidas con rojo para dar la detección por buena
        # (filtra falsos positivos de una sola imagen)
        self.frames_confirmacion = self.declare_parameter('frames_confirmacion', 3).value

        self.bridge = CvBridge()
        self.frames_con_rojo = 0

        self.pub_detectado = self.create_publisher(Bool, TOPIC_ROJO_DETECTADO, 10)
        self.pub_mascara = self.create_publisher(Image, TOPIC_ROJO_MASCARA, 10)
        self.create_subscription(Image, TOPIC_CAMARA, self.imagen_cb, qos_profile_sensor_data)

        self.get_logger().info(f'Escuchando {TOPIC_CAMARA}, publicando en {TOPIC_ROJO_DETECTADO}')

    def imagen_cb(self, msg: Image):
        try:
            imagen = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001  (imagen corrupta, encoding raro...)
            self.get_logger().warn(f'Error en cv_bridge: {e}')
            return

        # 1. Segmentar por color en HSV
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        mascara = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
            cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)
        pixeles_rojos = int(cv2.countNonZero(mascara))

        # 2. Filtro temporal sencillo: hay que verlo en varias imágenes seguidas
        if pixeles_rojos > self.umbral:
            self.frames_con_rojo += 1
        else:
            self.frames_con_rojo = 0
        detectado = self.frames_con_rojo >= self.frames_confirmacion

        # 3. Publicar SIEMPRE (también False). Así quien escucha sabe el estado
        #    actual y no solo "algún día vi algo rojo".
        self.pub_detectado.publish(Bool(data=detectado))

        # La máscara se publica como imagen en vez de usar cv2.imshow: un nodo
        # no debería abrir ventanas (puede ejecutarse en el robot, sin pantalla)
        mascara_msg = self.bridge.cv2_to_imgmsg(mascara, encoding='mono8')
        mascara_msg.header = msg.header
        self.pub_mascara.publish(mascara_msg)


def main():
    rclpy.init()
    nodo = DetectorRojo()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
