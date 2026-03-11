from setuptools import setup
from glob import glob
import os

package_name = 'multi_robot_scheduler'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simple659',
    maintainer_email='xiaowj2002@163.com',
    description='多机器人任务调度系统',
    license='MIT',
    # 注册节点：告诉 ROS2 这两个命令对应哪个 Python 函数的 main()
    entry_points={
        'console_scripts': [
            'scheduler_node = multi_robot_scheduler.scheduler_node:main',
            'robot_node     = multi_robot_scheduler.robot_node:main',
        ],
    },
)
