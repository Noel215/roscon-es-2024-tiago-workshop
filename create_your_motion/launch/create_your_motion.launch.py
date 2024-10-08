# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription

from launch_pal.include_utils import include_launch_py_description

from launch_ros.actions import Node


def generate_launch_description():

    tiago_sim = include_launch_py_description(
        pkg_name='tiago_gazebo',
        paths=['launch', 'tiago_gazebo.launch.py'],
        launch_arguments={
            'world_name': 'empty',
            'is_public_sim': 'True',
        }.items()
    )

    motion_builder = Node(
        package='play_motion_builder',
        executable='play_motion_builder_node',
        emulate_tty=True,
        output='both'
    )

    rqt_gui = Node(
        package='rqt_gui',
        executable='rqt_gui',
        emulate_tty=True,
        output='both'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(tiago_sim)
    ld.add_action(motion_builder)
    ld.add_action(rqt_gui)
    return ld
