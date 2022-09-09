'''
Author: Ke Zhang
Date: 2022-09-07 17:08:26
LastEditTime: 2022-09-07 17:12:18
Description: 
'''

from re import T
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='localization',
             executable='front_end',
             name='front_end',
             output='screen',
            #  parameters=[
            #     {"use_sim_time":True}
            #  ]
        )
    ])