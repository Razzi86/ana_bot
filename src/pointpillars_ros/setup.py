from setuptools import find_packages, setup

package_name = 'pointpillars_ros'

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
    maintainer='aidan',
    maintainer_email='arr160@pitt.edu',
    description='A ROS2 package for integrating PointPillars',
    license='Specify your license',  # Update your license information here
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointpillars_service = pointpillars_ros.pointpillars_service:main',  # Assuming your Python file is named pointpillars_service.py and your main function is main
        ],
    },
)
