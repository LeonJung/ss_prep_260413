import os
from glob import glob
from setuptools import setup

package_name = 'oa_viz'


def tree(dest_root, src_root):
    out = []
    for dirpath, _dirs, files in os.walk(src_root):
        if not files:
            continue
        dest = os.path.join(dest_root, dirpath)
        out.append((dest, [os.path.join(dirpath, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ] + tree('share/' + package_name, 'meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leon',
    maintainer_email='jung.ryuwoon@gmail.com',
    description='RViz model-vs-real pose comparison for OpenArm A2.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'joint_state_bridge = oa_viz.joint_state_bridge:main',
    ]},
)
