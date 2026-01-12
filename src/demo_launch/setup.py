from setuptools import setup

package_name = 'demo_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # ⚠ 必须匹配 Python 模块目录名 demo_launch
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Minimal demo launch package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
