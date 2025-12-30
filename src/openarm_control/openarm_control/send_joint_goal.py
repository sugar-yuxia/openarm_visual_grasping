import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class LeftArmTrajectorySender(Node):
    def __init__(self):
        super().__init__('left_arm_trajectory_sender')
        
        # 声明命令行参数：目标位置列表（默认值为原固定值）
        self.declare_parameter(
            'positions', 
            [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]  # 默认值
        )
        
        # 绑定左臂控制器
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info("等待左臂关节轨迹控制器...")
        self.action_client.wait_for_server()
        self.get_logger().info("已连接到左臂关节轨迹控制器")

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        # 左臂关节名称（根据实际调整）
        goal_msg.trajectory.joint_names = [
            "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
            "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
            "openarm_left_joint7"
        ]
        
        # 从命令行参数获取目标位置（关键修改）
        positions = self.get_parameter('positions').get_parameter_value().double_array_value
        # 校验参数长度（必须与关节数量一致）
        if len(positions) != len(goal_msg.trajectory.joint_names):
            self.get_logger().error(
                f"目标位置数量错误！需{len(goal_msg.trajectory.joint_names)}个，实际{len(positions)}个"
            )
            rclpy.shutdown()
            return
        
        # 设置轨迹点
        point = JointTrajectoryPoint()
        point.positions = positions  # 使用参数传入的位置
        point.time_from_start = Duration(sec=3, nanosec=0)
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"发送左臂目标位置：{positions}")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("左臂目标被拒绝！")
            return
        self.get_logger().info("左臂目标已接受，等待执行...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"左臂动作完成，结果: {result}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LeftArmTrajectorySender()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()