from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hoverboard_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('hoverboard_node', 'launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'hoverboard_driver = hoverboard_driver.hoverboard_driver:main',
            'manual_wheel_control = hoverboard_driver.manual_wheel_control:main',
            'test_rs485 = hoverboard_driver.test_rs485:main',
        ],
    },
) 