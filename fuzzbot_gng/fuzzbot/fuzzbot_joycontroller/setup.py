from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fuzzbot_joycontroller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nagashima',
    maintainer_email='nagashima@fuzzrobo.com',
    description='Arm Controller',
    license='FuzzRobo',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fuzzbot_joycontroller = fuzzbot_joycontroller.script.fuzzbot_joycontroller:main'
        ],
    },
)
