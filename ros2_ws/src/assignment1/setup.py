import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'assignment1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    # Do I need this? 
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomas',
    maintainer_email='thomas@todo.todo',
    description='TODO.',
    license='TODO: License declaration',

    # Entry points for console scripts
    entry_points={
        'console_scripts': [
            'wheel_publisher = assignment1.wheel_publisher:main',
        ],
    },
)
