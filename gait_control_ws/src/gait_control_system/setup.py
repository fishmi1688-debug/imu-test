from setuptools import setup
import os
from glob import glob

package_name = 'gait_control_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_data={
        package_name: ['model/*.onnx', 'model/*.pkl', 'model/*.keras', 'model/*.joblib'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'bleak>=0.22.1',
        'joblib',
    ],
    zip_safe=True,
    maintainer='gait_control_lab',
    maintainer_email='support@gaitcontrol.com',
    description='步态控制系统ROS2包',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'last3_optimized = gait_control_system.last3_optimized:main',
            'gait_control_node = gait_control_system.last3_optimized:main',
        ],
    },
)
