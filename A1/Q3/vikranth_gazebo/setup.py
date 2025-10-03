from setuptools import find_packages, setup

package_name = 'vikranth_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name, ['vikranth_gazebo/reactive_nav.py']),
        ('share/' + package_name + '/launch', [
            'launch/reactive_nav.launch.py',
            'launch/spa_nav.launch.py',
            'launch/hybrid_nav.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vikranth',
    maintainer_email='vikranth22541@iiitd.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spa_nav = vikranth_gazebo.spa_nav:main',
            'reactive_nav = vikranth_gazebo.reactive_nav:main',
            'hybrid_nav = vikranth_gazebo.hybrid_nav:main',
        ],
    },
)
