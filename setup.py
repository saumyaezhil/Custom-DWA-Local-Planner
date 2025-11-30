from setuptools import setup

package_name = 'dwa_local_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saumya',
    maintainer_email='saumya@example.com',
    description='Simple DWA Local Planner',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'simple_dwa = dwa_local_planner.dwa_local_planner_node:main',
        ],
    },
)
