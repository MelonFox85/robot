from setuptools import find_packages, setup

package_name = 'feedback_publisher'

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
    maintainer='Airat',
    maintainer_email='523ayrata@mail.ru',
    description='Package for feedback for control',
    license='Apache-2.0',
#    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lqr_publisher = feedback_publisher.lqr_publisher:main'
        ],
    },
)
