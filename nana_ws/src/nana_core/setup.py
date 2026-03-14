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
    install_requires=['pupil-apriltags',
                      'setuptools'],
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
            'griprpm_node = nana_core.griprpm_node:main',
            'apriltag_chanon = nana_core.apriltag_chanon:main',
            'plot_chanon = nana_core.plot_chanon:main',
            'Orchestrator_node = nana_core.Orchestrator_node:main',
            'apriltag_cameraPI_pub = nana_core.apriltag_cameraPI_pub:main',
            'apriltag_detector_sub = nana_core.apriltag_detector_sub:main',
            'plot_cameraPI_pub = nana_core.plot_cameraPI_pub:main',
            'plot_detector_sub = nana_core.plot_detector_sub:main',
            'apriltag_cameraPI_pub2 = nana_core.apriltag_cameraPI_pub2:main',
            'apriltag_detector_sub2 = nana_core.apriltag_detector_sub2:main',
            'Orchestrator_node2 = nana_core.Orchestrator_node2:main',
            'encoder_distance = nana_core.encoder_distance:main',
            'test = nana_core.test_encoder_pub:main',
        ],
    },
)
