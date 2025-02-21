from setuptools import find_packages, setup
import os
from glob import glob
# from Cython.Build import cythonize

package_name = 'astra_controller'

setup(
    # ext_modules=cythonize(package_name + "/*.py", compiler_directives={'language_level' : "3"}),
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=[
        'setuptools', 
        # 'Cython', 
    ],
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
            'teleop_web_node = astra_controller.teleop_web_node:main',
            'ik_node = astra_controller.ik_node:main',
            'dry_run_node = astra_controller.dry_run_node:main',
            'arm_node = astra_controller.arm_node:main',
            'lift_node = astra_controller.lift_node:main',
            'base_node = astra_controller.base_node:main',
            'head_node = astra_controller.head_node:main',
            'cam_node = astra_controller.cam_node:main',
        ],
    },
)
