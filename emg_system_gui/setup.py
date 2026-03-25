from setuptools import setup
from glob import glob
import os

package_name = 'emg_system_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meow',
    maintainer_email='tj6774@gmail.com',
    description='GUI launcher for EMG, Shadow Hand, and SO101 control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'config = emg_system_gui.config:main',
            'emg_system_gui = emg_system_gui.emg_system_gui:main',
            'main_window = emg_system_gui.main_window:main',
            'process_manager = emg_system_gui.process_manager:main',
            'ros_node = emg_system_gui.ros_node:main',
        ],
    },
)