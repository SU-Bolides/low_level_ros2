import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bolide_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bolide team',
    maintainer_email='baptiste.braun.delvoye@gmail.com',
    description='The package to use a simple teleoperation with the car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard=bolide_teleop.teleop_keyboard:main',
        ],
    },
)
