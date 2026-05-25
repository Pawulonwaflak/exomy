from setuptools import find_packages, setup

package_name = 'st3215'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['st3215', 'st3215.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='exomy',
    maintainer_email='exomy@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serwa = st3215.serwa:main',
            'wiele_pos = st3215.wiele_serw_pos:main',
            'wiele_vel = st3215.wiele_serw_vel:main',
            'sync_pos = st3215.Sync_Pos:main '        
            ],
    },
)
