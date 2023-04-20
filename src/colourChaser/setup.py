from glob import glob
import os
from setuptools import setup

package_name = 'colourChaser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/config', [
            "config/twist_mux_topics.yaml",
            "config/twist_mux_locks.yaml",
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasm-user',
    maintainer_email='kasm-user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'chaser = colourChaser.colourChaser:main',
                'roamer = colourChaser.roaming:main',
                'collision = colourChaser.collision:main',
        ],
    },
)
