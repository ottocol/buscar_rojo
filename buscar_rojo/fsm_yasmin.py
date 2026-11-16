#!/usr/bin/env python3
"""
Coordinador con máquina de estados (YASMIN 6.x, la versión que instala apt).

Tarea: deambular hasta ver algo rojo, hacer una foto e ir al punto (x, y).

  ACTIVAR_DEAMBULAR ──succeeded──▶ BUSCAR_ROJO ──detectado──▶ PARAR_DEAMBULAR
                                                                   │ succeeded
                                                                   ▼
  completada ◀──succeeded── IR_A_DESTINO ◀──succeeded/sin_foto── HACER_FOTO
                             │      ▲
                     aborted │      │ reintentar
                             ▼      │
                         COMPROBAR_INTENTOS ──agotados──▶ fallida

Fijaos en que ningún estado mueve el robot ni procesa imágenes: eso lo hacen
los nodos detector_rojo, deambulador y fotografo (que corren EN PARALELO, cada
uno en su proceso). Los estados solo:
  - llaman a servicios        (ServiceState)  → encender/apagar, hacer foto
  - esperan un evento         (estado propio) → "he visto rojo"
  - lanzan acciones largas    (ActionState)   → navegar con Nav2

Ejecutar (con capacidades.launch.py y Nav2 ya en marcha):
    ros2 run buscar_rojo fsm_yasmin --ros-args -p destino_x:=-10.0 -p destino_y:=-9.0
Ver la máquina en el navegador (http://localhost:5000):
    ros2 run yasmin_viewer yasmin_viewer_node
"""
import time
from threading import Event

import rclpy
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

import yasmin
from yasmin import Blackboard, CbState, State, StateMachine
from yasmin_ros import ActionState, ServiceState, set_ros_loggers
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub

from buscar_rojo.comun import (ACCION_NAVEGAR, SRV_DEAMBULAR, SRV_HACER_FOTO,
                               TOPIC_ROJO_DETECTADO, crear_pose)

# Outcomes propios (los de YASMIN son 'succeeded', 'aborted', 'canceled'...)
DETECTADO = 'detectado'
SIN_FOTO = 'sin_foto'
REINTENTAR = 'reintentar'
AGOTADOS = 'agotados'
# Outcomes finales de la máquina completa
COMPLETADA = 'completada'
FALLIDA = 'fallida'
CANCELADA = 'cancelada'

MAX_INTENTOS_NAVEGACION = 3


# ===========================================================================
# Estados
# ===========================================================================
class ActivarDeambular(ServiceState):
    """Llama a /deambulador/activar con data=True o data=False."""

    def __init__(self, activar: bool):
        self.activar = activar
        super().__init__(SetBool, SRV_DEAMBULAR, self.crear_peticion)

    def crear_peticion(self, blackboard: Blackboard) -> SetBool.Request:
        return SetBool.Request(data=self.activar)


class EsperarRojo(State):
    """Espera (sin bloquear al resto del sistema) a que el detector vea rojo.

    No hace falta un bucle con spin_once: el nodo de YASMIN (YasminNode) ya
    procesa los callbacks en otro hilo. El estado solo espera a que el
    callback "levante la bandera" (un threading.Event).
    """

    def __init__(self):
        super().__init__([DETECTADO, CANCEL])
        self.nodo = YasminNode.get_instance()
        self.rojo_visto = Event()

    def execute(self, blackboard: Blackboard) -> str:
        self.rojo_visto.clear()
        # Nos suscribimos al entrar en el estado y nos damos de baja al salir:
        # así no nos llegan mensajes "viejos" de antes de entrar
        sub = self.nodo.create_subscription(Bool, TOPIC_ROJO_DETECTADO, self.detector_cb, 10)
        yasmin.YASMIN_LOG_INFO('Buscando algo rojo...')
        try:
            self.rojo_visto.wait()
        finally:
            self.nodo.destroy_subscription(sub)

        if self.is_canceled():
            return CANCEL
        yasmin.YASMIN_LOG_INFO('¡Rojo detectado!')
        return DETECTADO

    def detector_cb(self, msg: Bool):
        if msg.data:
            self.rojo_visto.set()

    def cancel_state(self):
        self.rojo_visto.set()   # despierta al execute para que pueda terminar
        super().cancel_state()


class HacerFoto(ServiceState):
    """Llama a /fotografo/hacer_foto y guarda la ruta de la foto en el blackboard."""

    def __init__(self):
        super().__init__(Trigger, SRV_HACER_FOTO, self.crear_peticion,
                         outcomes=[SIN_FOTO], response_handler=self.procesar_respuesta)

    def crear_peticion(self, blackboard: Blackboard) -> Trigger.Request:
        return Trigger.Request()

    def procesar_respuesta(self, blackboard: Blackboard, respuesta: Trigger.Response) -> str:
        if not respuesta.success:
            yasmin.YASMIN_LOG_WARN(f'No se pudo hacer la foto: {respuesta.message}')
            return SIN_FOTO
        blackboard['foto'] = respuesta.message
        yasmin.YASMIN_LOG_INFO(f'Foto guardada en {respuesta.message}')
        return SUCCEED


class IrADestino(ActionState):
    """Envía a Nav2 el destino que hay en el blackboard (clave 'destino')."""

    def __init__(self):
        super().__init__(NavigateToPose, ACCION_NAVEGAR, self.crear_goal,
                         feedback_handler=self.feedback)
        self.n_feedback = 0

    def crear_goal(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        x, y = blackboard['destino']
        yasmin.YASMIN_LOG_INFO(f'Navegando a ({x}, {y})')
        goal = NavigateToPose.Goal()
        goal.pose = crear_pose(x, y)
        return goal

    def feedback(self, blackboard: Blackboard, feedback: NavigateToPose.Feedback):
        self.n_feedback += 1
        if self.n_feedback % 20 == 0:   # no inundar la consola
            yasmin.YASMIN_LOG_INFO(f'Faltan {feedback.distance_remaining:.2f} m')


def comprobar_intentos(blackboard: Blackboard) -> str:
    """Estado-función (CbState): ¿nos quedan intentos de navegación?"""
    blackboard['intentos'] += 1
    if blackboard['intentos'] >= MAX_INTENTOS_NAVEGACION:
        return AGOTADOS
    yasmin.YASMIN_LOG_WARN(f"Navegación fallida, reintento {blackboard['intentos']}")
    return REINTENTAR


# ===========================================================================
# Montaje de la máquina
# ===========================================================================
def crear_fsm() -> StateMachine:
    # handle_sigint=True: al pulsar Ctrl-C YASMIN cancela el estado activo
    sm = StateMachine(outcomes=[COMPLETADA, FALLIDA, CANCELADA], handle_sigint=True)

    sm.add_state('ACTIVAR_DEAMBULAR', ActivarDeambular(True),
                 transitions={SUCCEED: 'BUSCAR_ROJO', ABORT: FALLIDA})
    sm.add_state('BUSCAR_ROJO', EsperarRojo(),
                 transitions={DETECTADO: 'PARAR_DEAMBULAR', CANCEL: CANCELADA})
    sm.add_state('PARAR_DEAMBULAR', ActivarDeambular(False),
                 transitions={SUCCEED: 'HACER_FOTO', ABORT: FALLIDA})
    sm.add_state('HACER_FOTO', HacerFoto(),
                 # la foto no es imprescindible: si falla seguimos igualmente
                 transitions={SUCCEED: 'IR_A_DESTINO', SIN_FOTO: 'IR_A_DESTINO',
                              ABORT: 'IR_A_DESTINO'})
    sm.add_state('IR_A_DESTINO', IrADestino(),
                 transitions={SUCCEED: COMPLETADA, ABORT: 'COMPROBAR_INTENTOS',
                              CANCEL: CANCELADA})
    sm.add_state('COMPROBAR_INTENTOS', CbState([REINTENTAR, AGOTADOS], comprobar_intentos),
                 transitions={REINTENTAR: 'IR_A_DESTINO', AGOTADOS: FALLIDA})

    sm.set_start_state('ACTIVAR_DEAMBULAR')
    sm.validate()   # comprueba que todos los outcomes llevan a algún sitio
    return sm


def parar_deambulador(nodo):
    """Seguridad al salir: que el robot no se quede deambulando."""
    cliente = nodo.create_client(SetBool, SRV_DEAMBULAR)
    if cliente.wait_for_service(timeout_sec=1.0):
        futuro = cliente.call_async(SetBool.Request(data=False))
        for _ in range(20):          # esperar hasta 1 s; los callbacks los
            if futuro.done():        # procesa el hilo de YasminNode
                break
            time.sleep(0.05)


def main():
    rclpy.init()
    set_ros_loggers()   # los mensajes de YASMIN salen por el logger de ROS

    nodo = YasminNode.get_instance()
    # por defecto, esquina interna donde el cubo de basura
    destino_x = nodo.declare_parameter('destino_x', -2.0).value
    destino_y = nodo.declare_parameter('destino_y', -7.0).value

    sm = crear_fsm()
    visor = YasminViewerPub(sm, 'BUSCAR_ROJO')   # para verla en yasmin_viewer

    blackboard = Blackboard()
    blackboard['destino'] = (destino_x, destino_y)
    blackboard['intentos'] = 0

    try:
        resultado = sm(blackboard)   # bloquea hasta que la máquina termina
        yasmin.YASMIN_LOG_INFO(f'Tarea terminada: {resultado}')
    except Exception as e:  # noqa: BLE001
        # Con handle_sigint=True, Ctrl-C cancela el estado activo (p.ej. la
        # navegación) y la máquina termina lanzando una excepción
        yasmin.YASMIN_LOG_WARN(f'Tarea interrumpida: {e}')
    finally:
        parar_deambulador(nodo)      # por si acaso, no dejar el robot deambulando
        visor.shutdown()
        YasminNode.destroy_instance()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
