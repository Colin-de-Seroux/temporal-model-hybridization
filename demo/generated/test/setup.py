from setuptools import find_packages,setup

package_name = 'test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + pkgName]),
        ('share/'+pkgName, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto-generated',
    maintainer_email='noreply@example.com',
    description='Auto-generated ROS 2 package',
    license="Apache 2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = test.test:main',
            'subscribertest = test.subscriberTest:main'
        ],
    },
)