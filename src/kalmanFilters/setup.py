from setuptools import find_packages, setup

package_name = 'kalmanFilters'

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
    maintainer='wbforney',
    maintainer_email='wbforney@mtu.edu',
    description='Package that impements kalman filter, extended kalman filter, and unscented kalman filter with a turtlebot3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    'kalman = kalmanFilters.kalman_node:main',
            'covarCalc = kalmanFilters.covarianceCalculations:main'
        ],
    },
)
