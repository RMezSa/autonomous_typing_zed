import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'zed_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roberd',
    maintainer_email='rbdego@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zed_aruco_node = zed_aruco.zed_aruco_node:main',
            'typing_coordinator = zed_aruco.typing_coordinator:main',
            'calibration_probe = zed_aruco.calibration_probe:main',
            'fake_vision_publisher = zed_aruco.fake_vision_publisher:main',
            'fake_execute_key_server = zed_aruco.fake_execute_key_server:main'
        ],
    },
)
