import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final_proj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='ctighe@wpi.edu',
    description='Final project nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'basic_robot_control = final_proj.pos_example:main',
            'fwd_kin = final_proj.fwd_kin:main',
            'ik_node = final_proj.ik_node:main',
            'ik_client = final_proj.ik_client_node:main',
            'ik_service = final_proj.ik_service:main',    
        ],
    },
)
