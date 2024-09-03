from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'm0609_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "include"), glob('include/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paintingzoo',
    maintainer_email='paintingzoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm0609_controller_server=m0609_controller.server:main',
            'm0609_controller_client=m0609_controller.client:main',
            'm0609_trajectory_controller_client=m0609_controller.trajectory_client:main',
            'm0609_trajectory_controller_server=m0609_controller.move_plan_trajectory_server:main',
            'trajectory_plan_server=m0609_controller.trajectoryplan_server:main',
            'trajectory_plan_client=m0609_controller.trajectoryplan_client:main',
        ],
    },
)
