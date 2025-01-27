from setuptools import setup

import os
from glob import glob
package_name = 'mypkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shimodaira',
    maintainer_email='s23C1069@s.chibakoudai.jp',
    description='ロボットシステム学のサンプル',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sunrise_sunset = mypkg.sunrise_sunset:main'
        ],
    },
)
