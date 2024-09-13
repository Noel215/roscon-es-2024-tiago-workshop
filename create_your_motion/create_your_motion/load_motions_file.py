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

import argparse
import os
import yaml

import rclpy

from play_motion2_msgs.msg import Motion
from play_motion2_msgs.srv import AddMotion


def read_yaml(yaml_file):
    with open(yaml_file, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None


def check_file_exists(file_path):
    return os.path.isfile(file_path)


def create_motion_msg(key, motion):
    motion_msg = Motion()
    motion_msg.key = key

    motion_msg.joints = motion['joints']
    motion_msg.positions = motion['positions']
    motion_msg.times_from_start = motion['times_from_start']

    try:
        motion_msg.name = motion['meta']['name']
        motion_msg.description = motion['meta']['description']
        motion_msg.usage = motion['meta']['usage']
    except KeyError:
        pass

    return motion_msg


def add_motion(node, client, motion_msg):
    request = AddMotion.Request()
    request.motion = motion_msg
    request.overwrite = True

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print(f'Motion {motion_msg.key} added')
    else:
        print(f'Error adding motion {motion_msg.key}')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('motions_yaml_file', help='Path to the yaml file with the motions')
    args = parser.parse_args()

    rclpy.init(args=None)

    if not check_file_exists(args.motions_yaml_file):
        print('File not found')
        return

    yaml_content = read_yaml(args.motions_yaml_file)
    try:
        motions = yaml_content['/play_motion2']["ros__parameters"]["motions"]
    except KeyError:
        print('No motions defined for /play_motion2')
        return

    node = rclpy.create_node('play_motion2_file_loader')
    add_motion_client = node.create_client(AddMotion, '/play_motion2/add_motion')

    if not add_motion_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('/play_motion2/add_motion service not available')
        return

    for key, motion in motions.items():
        motion_msg = create_motion_msg(key, motion)
        add_motion(node, add_motion_client, motion_msg)

    node.destroy_node()
    rclpy.shutdown()
