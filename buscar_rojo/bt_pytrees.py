#!/usr/bin/env python3
"""
Coordinador con behavior tree (py_trees 2.6 / py_trees_ros 2.6).

Misma tarea que fsm_yasmin.py: deambular hasta ver rojo, foto, ir a (x, y).

    [o] Raíz  (Parallel, éxito cuando acabe "Misión")
     ├── Rojo → BB          (subscriber: copia /detector_rojo/detectado al blackboard)
     └── [-] Misión         (Sequence con memoria: un hijo detrás de otro)
          ├── [o] Buscar rojo   (Parallel: éxito en cuanto UN hijo tenga éxito)
          │    ├── Deambular       (RUNNING siempre; enciende/apaga el deambulador)
          │    └── ¿Rojo?          (RUNNING hasta que rojo_detectado == True)
          ├── Hacer foto        (cliente del servicio /fotografo/hacer_foto)
          └── Reintentar x3     (decorador Retry)
               └── Ir a destino    (cliente de la acción /navigate_to_pose)

Diferencias con la FSM que conviene mirar:
  - "deambular hasta ver rojo" es un Parallel de dos hijos: no hace falta
    un estado para encender y otro para apagar. Cuando ¿Rojo? tiene éxito el
    Parallel termina y py_trees llama a terminate() de Deambular, que apaga.
  - Los datos de los sensores entran al blackboard en cada tick por la rama
    "Rojo → BB" y las condiciones solo leen el blackboard.
  - El árbol se "tickea" 10 veces por segundo: cada tick recorre el árbol y
    cada nodo devuelve RUNNING, SUCCESS o FAILURE.

Ejecutar (con capacidades.launch.py y Nav2 ya en marcha):
    ros2 run buscar_rojo bt_pytrees --ros-args -p destino_x:=-10.0 -p destino_y:=-9.0
Ver el árbol en vivo en otro terminal:
    ros2 run py_trees_ros py-trees-tree-watcher
"""
import operator
import time

import py_trees
import py_trees_ros
import rclpy
from nav2_msgs.action import NavigateToPose
from py_trees.common import Status
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from buscar_rojo.comun import (ACCION_NAVEGAR, SRV_DEAMBULAR, SRV_HACER_FOTO,
                               TOPIC_ROJO_DETECTADO, crear_pose)


# ===========================================================================
# Comportamiento propio: Deambular
# ===========================================================================
class Deambular(py_trees.behaviour.Behaviour):
    """Mantiene activado el nodo deambulador mientras este comportamiento esté RUNNING.

    Ciclo de vida de un comportamiento en py_trees:
      setup()      una vez, al arrancar el árbol      → crear el cliente ROS
      initialise() cada vez que empieza a ejecutarse   → activar el deambulador
      update()     en cada tick                        → devolver el estado
      terminate()  cuando acaba o lo interrumpen       → desactivar el deambulador
    """

    def __init__(self, name: str = 'Deambular'):
        super().__init__(name)
        self.cliente = None
        self.futuro = None

    def setup(self, **kwargs):
        nodo = kwargs['node']   # py_trees_ros pasa el nodo ROS del árbol
        self.cliente = nodo.create_client(SetBool, SRV_DEAMBULAR)
        if not self.cliente.wait_for_service(timeout_sec=10.0):
            raise py_trees_ros.exceptions.TimedOutError(f'no encuentro {SRV_DEAMBULAR}')

    def initialise(self):
        self.futuro = self.cliente.call_async(SetBool.Request(data=True))

    def update(self) -> Status:
        if self.futuro.done() and not self.futuro.result().success:
            return Status.FAILURE
        # Deambular nunca "termina" por sí mismo: es el Parallel padre quien lo para
        self.feedback_message = 'deambulando'
        return Status.RUNNING

    def terminate(self, new_status: Status):
        # Se llama al pasar a SUCCESS/FAILURE o al ser interrumpido (INVALID)
        if self.cliente is not None and self.cliente.service_is_ready():
            self.cliente.call_async(SetBool.Request(data=False))
        self.feedback_message = 'parado'


# ===========================================================================
# Montaje del árbol
# ===========================================================================
def crear_arbol(destino_x: float, destino_y: float) -> py_trees.behaviour.Behaviour:
    # --- Rama de datos: topic → blackboard -------------------------------
    rojo_a_bb = py_trees_ros.subscribers.ToBlackboard(
        name='Rojo → BB',
        topic_name=TOPIC_ROJO_DETECTADO,
        topic_type=Bool,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={'rojo_detectado': 'data'},   # clave: campo del mensaje
        initialise_variables={'rojo_detectado': False},
        # Mantener el último valor aunque no llegue mensaje en algún tick
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )

    # --- Buscar rojo: deambular EN PARALELO con la comprobación -----------
    hay_rojo = py_trees.behaviours.WaitForBlackboardVariableValue(
        name='¿Rojo?',
        check=py_trees.common.ComparisonExpression(
            variable='rojo_detectado', value=True, operator=operator.eq),
    )
    buscar = py_trees.composites.Parallel(
        name='Buscar rojo',
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[Deambular(), hay_rojo],
    )

    # --- Hacer foto: cliente de servicio -----------------------------------
    hacer_foto = py_trees_ros.service_clients.FromConstant(
        name='Hacer foto',
        service_type=Trigger,
        service_name=SRV_HACER_FOTO,
        service_request=Trigger.Request(),
        key_response='respuesta_foto',   # la respuesta queda en el blackboard
    )

    # --- Ir a destino: cliente de acción Nav2, con reintentos -------------
    goal = NavigateToPose.Goal()
    goal.pose = crear_pose(destino_x, destino_y)
    ir_a_destino = py_trees_ros.action_clients.FromConstant(
        name='Ir a destino',
        action_type=NavigateToPose,
        action_name=ACCION_NAVEGAR,
        action_goal=goal,
        generate_feedback_message=lambda fb: f'faltan {fb.feedback.distance_remaining:.2f} m',
    )
    reintentar = py_trees.decorators.Retry('Reintentar x3', ir_a_destino, num_failures=3)

    # --- La misión: secuencia con memoria -----------------------------------
    # memory=True: los hijos que ya tuvieron éxito no se vuelven a ejecutar
    mision = py_trees.composites.Sequence(
        name='Misión', memory=True, children=[buscar, hacer_foto, reintentar])

    # --- Raíz: datos + misión en paralelo -----------------------------------
    # synchronise=False para que "Rojo → BB" se siga ejecutando en todos los ticks
    raiz = py_trees.composites.Parallel(
        name='Raíz',
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[mision], synchronise=False),
        children=[rojo_a_bb, mision],
    )
    return raiz


def main():
    # Igual que en la FSM: sin el manejador de Ctrl-C de rclpy para poder
    # parar el robot ordenadamente antes de salir
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = rclpy.create_node('bt_buscar_rojo')
    destino_x = nodo.declare_parameter('destino_x', -10.0).value
    destino_y = nodo.declare_parameter('destino_y', 10.0).value

    arbol = py_trees_ros.trees.BehaviourTree(crear_arbol(destino_x, destino_y))
    try:
        # setup() llama al setup() de todos los nodos; espera a los servidores
        arbol.setup(node=nodo, timeout=15.0)
    except Exception as e:  # noqa: BLE001  (timeout esperando servicios/acciones)
        nodo.get_logger().error(f'Fallo en setup del árbol: {e}')
        rclpy.try_shutdown()
        return

    # Mostrar el árbol en consola cada vez que cambie
    arbol.add_post_tick_handler(mostrar_si_cambia())
    arbol.tick_tock(period_ms=100)   # 10 ticks por segundo (con un timer de ROS)

    try:
        while rclpy.ok() and arbol.root.status not in (Status.SUCCESS, Status.FAILURE):
            rclpy.spin_once(nodo, timeout_sec=0.1)
        nodo.get_logger().info(f'Tarea terminada: {arbol.root.status.value}')
        respuesta = py_trees.blackboard.Blackboard.storage.get('/respuesta_foto')
        if respuesta is not None:
            nodo.get_logger().info(f'Foto: {respuesta.message}')
    except KeyboardInterrupt:
        # Parar el árbol: llama a terminate() de los nodos RUNNING, así que
        # Deambular apaga el deambulador y el cliente de Nav2 cancela el goal
        arbol.root.stop(Status.INVALID)
        time.sleep(0.5)       # dar tiempo a que salgan las peticiones
    finally:
        arbol.shutdown()
        rclpy.try_shutdown()


def mostrar_si_cambia():
    """Devuelve un post-tick handler que imprime el árbol cuando cambia algún estado."""
    anterior = {'estados': None}

    def handler(arbol):
        estados = [(b.name, b.status) for b in arbol.root.iterate()]
        if estados != anterior['estados']:
            print(py_trees.display.unicode_tree(arbol.root, show_status=True))
            anterior['estados'] = estados
    return handler


if __name__ == '__main__':
    main()
