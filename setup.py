from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ais_3dof'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),
         glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='krohnkg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = ais2105_3dof.pub_servo_angle:main',
                'compute = ais2105_3dof.main:main',
                'kin_calc = 3dof_kin.kin_calc:main',
                'test_prg = 3dof_kin.kin_test:main',
        ],
    },
)
