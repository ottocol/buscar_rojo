"""
Constantes y utilidades compartidas por todos los nodos del paquete.

Tener los nombres de topics y servicios en un único sitio evita el error
más típico al repartir una tarea en varios nodos: que un nodo publique en
'/detector_rojo/detectado' y el otro escuche en '/detector/detectado'.
"""
import math

from geometry_msgs.msg import PoseStamped

# ---------------------------------------------------------------------------
# Topics de los sensores (robot simulado en mvsim)
# ---------------------------------------------------------------------------
TOPIC_CAMARA = '/camera1_image'
TOPIC_LASER = '/scan'
TOPIC_VEL = '/cmd_vel'

# ---------------------------------------------------------------------------
# Interfaz de nuestros propios nodos
# ---------------------------------------------------------------------------
# detector_rojo publica continuamente si ve (True) o no (False) algo rojo
TOPIC_ROJO_DETECTADO = '/detector_rojo/detectado'   # std_msgs/Bool
TOPIC_ROJO_MASCARA = '/detector_rojo/mascara'       # sensor_msgs/Image (depuración)

# deambulador: se activa / desactiva con un servicio
SRV_DEAMBULAR = '/deambulador/activar'              # std_srvs/SetBool

# fotografo: guarda en disco la última imagen de la cámara
SRV_HACER_FOTO = '/fotografo/hacer_foto'            # std_srvs/Trigger

# Nav2: acción para navegar a un punto del mapa
ACCION_NAVEGAR = '/navigate_to_pose'                # nav2_msgs/action/NavigateToPose


def crear_pose(x: float, y: float, yaw: float = 0.0, frame: str = 'map') -> PoseStamped:
    """Construye un PoseStamped en el plano a partir de (x, y, yaw).

    La orientación en ROS es un cuaternio. Para un giro de 'yaw' radianes
    alrededor del eje Z el cuaternio es (0, 0, sin(yaw/2), cos(yaw/2)).
    """
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose
