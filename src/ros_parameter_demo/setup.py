from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_parameter_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']) + [
        "workflow.VitesseAPI.simulator",
        "workflow.VitesseAPI.libraries",
        "workflow.AscanProcessingModule.objects.x86_64"
    ],
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name),
            ['workflow/config.json']),
        (os.path.join('lib', package_name, 'VitesseAPI', 'libraries'),
            glob('workflow/VitesseAPI/libraries/*')),
        (os.path.join('lib', package_name, 'VitesseAPI', 'simulator'),
            glob('workflow/VitesseAPI/simulator/*')),
        (os.path.join('lib', package_name, 'AscanProcessingModule', 'objects', 'x86_64'),
            glob('workflow/AscanProcessingModule/objects/x86_64/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuxuan Long',
    maintainer_email='contact@sonobotics.com',
    description='ROS 2 Demo Package for Vitesse Devices',
    license='ALL_RIGHTS_RESERVED',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor = workflow.processor:main',
            'publisher = workflow.publisher:main',
            'controller = workflow.controller:main',
        ],
    },
)
