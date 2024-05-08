from setuptools import find_packages, setup

package_name = '3dof_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maggebag',
    maintainer_email='magnus.mortensen@hotmail.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracking = 3dof_camera.balltracking:main',
            'visualiser = 3dof_camera.visualiser:main',
            'convert_to_centi = 3dof_camera.image_plane_calc:main'
        ],
    },
)
