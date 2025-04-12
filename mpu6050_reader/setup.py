from setuptools import find_packages, setup

package_name = 'mpu6050_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2', 'lgpio', 'matplotlib', 'numpy', 'math'],
    zip_safe=True,
    maintainer='airat',
    maintainer_email='523ayrata@mail.ru',
    description='ROS2 nodes for reading, filtering and visualising data from MPU6050',
    license='Apache License 2.0',
#    extras_require={
#           'test': ['required_test_library>=1.0'],
    entry_points={
        'console_scripts': [
            'mpu6050_node = mpu6050_reader.mpu6050_node:main',
            'mpu6050_filter = mpu6050_reader.mpu6050_filter:main',
            'mpu6050_graphs = mpu6050_reader.mpu6050_graphs:main',
            'mpu6050_graphs_plotter = mpu6050_reader.mpu6050_graphs_plotter:main'
        ],
    },
)
