from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odroid_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'odroid_driver = odroid_driver.odroid_driver:main',
            'cmd_vel_mux_node = odroid_driver.cmd_vel_mux_node:main',
        ],
    },
)

