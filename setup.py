from setuptools import setup, find_packages

package_name = 'controlbox_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/config', ['config/hardware_params_9chambers.yaml']),
        # ('share/' + package_name + '/config', ['config/serial_com.yaml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['launch/controlbox_bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eldiez',
    maintainer_email='diegobianchi@live.it',
    description='ROS2 node for bridging the control box with ROS via serial communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controlbox_bridge_node = controlbox_bridge.main:main',
        ],
    },
)
