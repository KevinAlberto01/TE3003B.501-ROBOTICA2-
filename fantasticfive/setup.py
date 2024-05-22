from setuptools import find_packages, setup

package_name = 'fantasticfive'

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
    maintainer='puzzlebot',
    maintainer_email='puzzlebot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pathGenerator = fantasticfive.path_generator:main",
            "controller = fantasticfive.controller:main",
            "localisation = fantasticfive.localisation:main",
            "pruebas = fantasticfive.prueba:main"

        ],
    },
)
