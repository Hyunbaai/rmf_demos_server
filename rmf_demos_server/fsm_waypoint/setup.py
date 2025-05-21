from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fsm_waypoint'

setup(
    name=package_name,
    version='0.0.1',
    # packages=[package_name],
    packages=find_packages(exclude=['test']), # 폴더 모두
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Bcchoi',
    author_email='zaxrok@gmail.com',
    maintainer='bcchoi',
    maintainer_email='zaxroK@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Receive PVD data and send it to the Kafka broker.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_waypoint_node=fsm_waypoint.fsm_waypoint_node:main',
        ],
    },
    package_data={package_name: ['logger2.py']},
)
