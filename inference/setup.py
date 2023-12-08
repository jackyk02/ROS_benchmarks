from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('inference/*.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackyk',
    maintainer_email='jackykwok@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = inference.talker:main',
            'listener = inference.listener:main',
        ],
    },
)
