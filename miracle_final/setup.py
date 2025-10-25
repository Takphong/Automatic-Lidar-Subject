from setuptools import find_packages, setup

package_name = 'miracle_final'

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
    maintainer='vulpes',
    maintainer_email='vulpes@todo.todo',
    description='Final exam package with three nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sectiona = miracle_final.sectiona:main',
            'sectionb = miracle_final.sectionb:main',
            'sectionc = miracle_final.sectionc:main',
        ],
    },
)
