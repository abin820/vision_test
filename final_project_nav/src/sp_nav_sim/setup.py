import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sp_nav_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'sp_nav_sim.sim': ['_dyn_core*.so'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pygame', 'numpy', 'Pillow'],
    zip_safe=False,
    maintainer='rm',
    maintainer_email='rm@example.com',
    description='SP Nav tutorial simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_robot = sp_nav_sim.sim_robot_node:main',
        ],
    },
)
