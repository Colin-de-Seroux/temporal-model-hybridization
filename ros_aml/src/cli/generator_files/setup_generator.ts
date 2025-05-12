import { Model } from '../../language/generated/ast.js';
import { camelCaseToSnakeCase } from '../cli-util.js';

export function generateSetupPy(pkgName: string, model: Model): string {
    return `
import os 
from glob import glob 
from setuptools import find_packages,setup

package_name = '${pkgName}'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
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
            ${model.nodes.map((n) => `'${camelCaseToSnakeCase(n.name)} = ${pkgName}.${camelCaseToSnakeCase(n.name)}:main'`).join(',\n            ')}
        ],
    },
)
`.trim();
}

export function generateSetupCfg(pkgName: string): string {
    return `
[develop]
script_dir=$base/lib/${pkgName}
[install]
install_scripts=$base/lib/${pkgName}
`.trim();
}
