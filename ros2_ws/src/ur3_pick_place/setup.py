from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur3_pick_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.stl')),
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachu',
    maintainer_email='your.email@example.com',
    description='UR3 Pick and Place Demo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_node = ur3_pick_place.pick_place_node:main',
            'spawn_objects_node = ur3_pick_place.spawn_objects:main',
            'trajectory_visualizer = ur3_pick_place.scripts.trajectory_visualizer:main',
            'ur3_color_enhancer = ur3_pick_place.scripts.ur3_color_enhancer:main',
        ],
    },
)