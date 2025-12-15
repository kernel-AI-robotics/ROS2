from setuptools import setup

package_name = 'dr_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # urdf / launch 추가
        ('share/' + package_name + '/urdf', [
            'urdf/e0509_with_gripper.xacro',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/display_e0509_with_gripper.launch.py',
        ]),
        #Rviz 설정 추가
        ('share/' + package_name + '/rviz',[
       	    'rviz/e0509_with_gripper.rviz',
       	]),
    ],
    install_requires=['setuptools', 'xacro'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='you@example.com',
    description='Doosan e0509 + RH-P12-RN-A combined URDF',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
