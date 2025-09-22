from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'custom_code'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_tf = custom_code.my_tf:main",
            "likelihood = custom_code.likelihood:main",
            "localization = custom_code.my_localization:main",
        ],
    },
)
