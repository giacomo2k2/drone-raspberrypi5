from setuptools import find_packages, setup

package_name = 'status_monitoring'

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
    maintainer='giacomo',
    maintainer_email='pezzolati.g@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_monitoring_node= status_monitoring.status_monitoring_node:main',
            'shutdown_node= status_monitoring.shutdown_node:main',
        ],
    },
)
