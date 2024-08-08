from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astra_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='i@18kas.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_relay_node = astra_controller.moveit_relay_node:main',
            'teleop_node = astra_controller.teleop_node:main',
            'ik_node = astra_controller.ik_node:main',
            'dry_run_node = astra_controller.dry_run_node:main',
            'arm_node = astra_controller.arm_node:main',
            'lift_node = astra_controller.lift_node:main',
        ],
    },
)
