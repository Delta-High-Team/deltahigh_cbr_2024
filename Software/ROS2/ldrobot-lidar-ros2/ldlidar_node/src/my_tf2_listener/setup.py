from setuptools import find_packages, setup

package_name = 'my_tf2_listener'

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
    maintainer='deltahigh',
    maintainer_email='joaodearaujoduarte@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'tf2_listener = my_tf2_listener.tf2_listener:main',
        'tf_send_yaw = my_tf2_listener.tf_send_yaw:main',
        'tf_send_position = my_tf2_listener.tf_send_position:main',
        'position = my_tf2_listener.position:main',
        'gps_send_position = my_tf2_listener.gps_send_position:main',
        ],
    },
)
