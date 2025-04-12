from setuptools import setup

package_name = 'custom_interfaces'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['action/SetMotorSpeed.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Airat',
    maintainer_email='523ayrata@mail.ru',
    description='Custom action interfaces for ROS2 LQR control',
    license='Apache License 2.0',
#    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
