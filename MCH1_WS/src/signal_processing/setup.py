from setuptools import setup
import os
from glob import glob

package_name = 'signal_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humberto',
    maintainer_email='humberto@todo.todo',
    description='Signal Processing Nodes in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_generator = signal_processing.signal_generator:main',
            'process = signal_processing.process:main',
        ],
    },
)
