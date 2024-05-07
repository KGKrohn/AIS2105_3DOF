from setuptools import find_packages, setup

package_name = '3dof_kin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='maggebag',
    maintainer_email='magnus.mortensen@hotmail.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'kin_calc = 3dof_kin.kin_calc:main',
                'test_prg = 3dof_kin.kin_test:main',
        ],
    },
)
