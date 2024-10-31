from setuptools import find_packages, setup

package_name = 'ibvs'

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
    maintainer='Dylan Fanner',
    maintainer_email='dylanfanner@gmail.com',
    description='Offboard control package for maritime landing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ibvs_node = ibvs.ibvs_node:main'
        ],
    },
)
