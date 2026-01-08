import os
from glob import glob
from setuptools import setup

package_name = 'fleet_adapter_sony'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob(f'{package_name}/launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        # Runtime deps used in the adapter code
        'PyYAML',
        'numpy',
        'nudged',
        # gRPC is only required when sony.grpc is enabled in config.yaml,
        # but listing it here makes "real Sony" tests smoother.
        'grpcio',
    ],
    zip_safe=True,
    maintainer='fleet_adapter_sony',
    maintainer_email='devnull@example.com',
    description='RMF fleet adapter for Sony FMS (gRPC) with a simulation fallback backend.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_sony.fleet_adapter:main',
        ],
    },
)
