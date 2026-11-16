import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'buscar_rojo'

def ficheros(patron):
    """Solo ficheros (data_files no admite carpetas como __pycache__)."""
    return [f for f in glob(patron) if os.path.isfile(f)]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/world', ficheros('world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robots Móviles UA',
    maintainer_email='otto@ua.es',
    description='Buscar rojo, hacer foto e ir a un punto: FSM (YASMIN) y BT (py_trees)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Nodos "capacidad" (cada uno hace UNA cosa)
            'detector_rojo = buscar_rojo.detector_rojo:main',
            'deambulador   = buscar_rojo.deambulador:main',
            'fotografo     = buscar_rojo.fotografo:main',
            # Navegación por código con nav2_simple_commander
            'pose_inicial  = buscar_rojo.pose_inicial:main',
            'ir_a_punto    = buscar_rojo.ir_a_punto:main',
            # Coordinadores (usad uno u otro, no los dos a la vez)
            'fsm_yasmin    = buscar_rojo.fsm_yasmin:main',
            'bt_pytrees    = buscar_rojo.bt_pytrees:main',
        ],
    },
)
