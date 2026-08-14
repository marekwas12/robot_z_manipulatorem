import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/camera_bridge']),
        ('share/camera_bridge', ['package.xml']),
        (os.path.join('share', 'camera_bridge', 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marek',
    maintainer_email='marek@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = camera_bridge.camera_node:main',
            'stereo_processor = camera_bridge.camera_processor:main',
            'dual_camera_node = camera_bridge.dual_camera_node:main',
        ],
    },
)
