from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
# 修正：从launch_ros.substitutions导入FindPackageShare
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 构建MoveIt配置
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_moveit_config"
    ).to_moveit_configs()

    # 手动拼接RViz配置文件路径
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("openarm_moveit_config"), "config", "moveit.rviz"]
    )

    # 1. 启动move_group节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.robot_description_semantic,  # 新增：加载SRDF
            {
                "planning_scene_monitor.publish_geometry_updates": True,
                "planning_scene_monitor.publish_state_updates": True,
                "planning_scene_monitor.publish_transforms_updates": True,
                "planning_scene_monitor.monitor_dynamics": False,
            }
        ],
    )

    # 2. 启动RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    # 3. 启动机器人状态发布器
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    # 4. 启动关节状态发布器
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        move_group_node,
        rviz_node
    ])
