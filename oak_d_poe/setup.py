import os
from glob import glob
from setuptools import setup

package_name = 'oak_d_poe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vbaia23046',
    maintainer_email='vbaia23046@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_publisher = oak_d_poe.rgb_publisher:main',
            'rgb_subscriber = oak_d_poe.rgb_subscriber:main',
            'implement_detector = oak_d_poe.implement_detector:main',
        ],
    },
)
