#!/usr/bin/env python3
"""
Nodo fotografo: guarda en disco la imagen actual de la cámara cuando se le pide.

    /camera1_image (Image) ──▶ [fotografo]
    /fotografo/hacer_foto (std_srvs/Trigger) ──▶ guarda ~/fotos_robot/foto_AAAAMMDD_HHMMSS.png

Es una *acción puntual*: se le pide, la hace y responde. Por eso encaja con un
servicio (petición → respuesta rápida) y no con un topic ni con una acción.

Probarlo a mano:
    ros2 run buscar_rojo fotografo
    ros2 service call /fotografo/hacer_foto std_srvs/srv/Trigger
"""
import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from buscar_rojo.comun import SRV_HACER_FOTO, TOPIC_CAMARA


class Fotografo(Node):

    def __init__(self):
        super().__init__('fotografo')
        carpeta = self.declare_parameter('carpeta', '~/fotos_robot').value
        self.carpeta = os.path.expanduser(carpeta)
        os.makedirs(self.carpeta, exist_ok=True)

        self.bridge = CvBridge()
        self.ultima_imagen = None   # nos quedamos solo con la última

        self.create_subscription(Image, TOPIC_CAMARA, self.imagen_cb, qos_profile_sensor_data)
        self.create_service(Trigger, SRV_HACER_FOTO, self.hacer_foto_cb)
        self.get_logger().info(f'Listo. Las fotos se guardan en {self.carpeta}')

    def imagen_cb(self, msg: Image):
        self.ultima_imagen = msg

    def hacer_foto_cb(self, peticion: Trigger.Request, respuesta: Trigger.Response):
        if self.ultima_imagen is None:
            respuesta.success = False
            respuesta.message = 'Todavía no he recibido ninguna imagen'
            return respuesta

        imagen = self.bridge.imgmsg_to_cv2(self.ultima_imagen, desired_encoding='bgr8')
        nombre = datetime.now().strftime('foto_%Y%m%d_%H%M%S.png')
        ruta = os.path.join(self.carpeta, nombre)
        respuesta.success = bool(cv2.imwrite(ruta, imagen))
        respuesta.message = ruta if respuesta.success else f'No se pudo escribir {ruta}'
        self.get_logger().info(f'Foto: {respuesta.message}')
        return respuesta


def main():
    rclpy.init()
    nodo = Fotografo()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
