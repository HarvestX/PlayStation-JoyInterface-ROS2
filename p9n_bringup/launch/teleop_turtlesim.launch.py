# Copyright 2023 HarvestX Inc.
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
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description."""
    hw_type_arg = DeclareLaunchArgument(
        'hw_type', default_value=TextSubstitution(text='DualSense'))
    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value=TextSubstitution(text='/turtle1/cmd_vel'))

    joy_container = ComposableNodeContainer(
        name='joy_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy',
                namespace='',
            ),
            ComposableNode(
                package='p9n_node',
                plugin='p9n_node::TeleopTwistJoyNode',
                name='teleop_twist_joy_node',
                namespace='',
                parameters=[{
                        'hw_type': LaunchConfiguration('hw_type')
                }],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('topic_name'))
                ],
            )
        ],
    )

    turtlesim = Node(
            name='turtlesim',
            package='turtlesim',
            executable='turtlesim_node'
        )

    ld = LaunchDescription()

    ld.add_action(hw_type_arg)
    ld.add_action(topic_name_arg)

    ld.add_action(joy_container)
    ld.add_action(turtlesim)

    return ld
