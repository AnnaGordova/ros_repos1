from setuptools import find_packages, setup

package_name = 'nav_in_lab'

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
    maintainer='anna',
    maintainer_email='annagordova2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "go_lab = nav_in_lab.go_lab:main",
            "go_lab_serv = nav_in_lab.go_lab_serv:main"
        ],
    },
)