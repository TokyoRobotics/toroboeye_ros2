#! /usr/bin/python

from click import launch
from setuptools import setup
import os
from glob import glob

package_name = 'toroboeye_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tokyo Robotics Inc.',
    maintainer_email='please_contact_us_via_the_bugtracker_site_of_url_tag@dummy.address',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = toroboeye_camera.toroboeye_camera:main',
        ],
    },
)
