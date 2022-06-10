# Copyright 2022 HarvestX Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description."""
    launch_args = [
        DeclareLaunchArgument(
            'hw_type',
            default_value=TextSubstitution(text='DualSense')
        )
    ]
    nodes = [
        ComposableNodeContainer(
            name='joy_container',
            namespace='p9n',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='joy',
                    plugin='joy::Joy',
                    name='joy',
                ),
                ComposableNode(
                    package='p9n_test',
                    plugin='p9n_test::PlayStationTestNode',
                    name='p9n_test',
                    parameters=[{
                        'hw_type': LaunchConfiguration('hw_type')
                    }]
                ),
            ])]

    return LaunchDescription(launch_args + nodes)
