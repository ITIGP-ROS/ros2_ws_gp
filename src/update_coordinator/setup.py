from setuptools import find_packages, setup

package_name = 'update_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/update_coordinator.yaml']),
        ('share/' + package_name + '/launch', ['launch/update_coordinator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='youhana',
    author_email='youhanabeshay@gmail.com',
    maintainer='youhana',
    maintainer_email='youhanabeshay@gmail.com',
    description='ROS2 node for orchestrating SecOC-authenticated CAN-based ECU firmware updates',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'update_coordinator = update_coordinator.node:main',
        ],
    },
)
