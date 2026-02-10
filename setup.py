from setuptools import find_packages, setup

package_name = 'so101'

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
    maintainer='gkim451',
    maintainer_email='gkim451@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'so101 = so101.so101:main',
            'so101_real_control = so101.so101_real_control:main',
            'so101_right = so101.so101_right:main',
            'so101_real_control_right = so101.so101_real_control_right:main'
        ],
    },
)
