from glob import glob

from setuptools import find_packages, setup

package_name = 'pick_place_damsssss'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='classic control team',
    maintainer_email='todo@example.com',
    description='Classic-control pick and place of a cube, plus success-rate evaluation.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'pick_place = pick_place_damsssss.pick_place_node:main',
            'evaluate = pick_place_damsssss.evaluate:main',
        ],
    },
)
