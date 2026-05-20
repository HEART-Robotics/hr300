from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hr300_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),  # сервис
        ('share/' + package_name + '/meshes/stl', glob('meshes/stl/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='likerobotics',
    maintainer_email='xmlpro100@gmail.com',
    description='Visualization and Robot Model for robot manipulator HR-300 using ROS2 framework.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'motion_demo_node = hr300_description.motion_demo:main',
            'fk_service = hr300_description.fk_service:main',
        ],
    },
)
