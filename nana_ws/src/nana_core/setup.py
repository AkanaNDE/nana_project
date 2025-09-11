from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nana_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadeem',
    maintainer_email='nadeem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driverpm_node = nana_core.driverpm_node:main',
            'joystick_node = nana_core.joystick_node:main',
            'armrpm_node = nana_core.armrpm_node:main',
            'mahorpm_node = nana_core.mahorpm_node:main',
        ],
    },
)
