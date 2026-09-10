import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
import math

# 导入 Action 的底层的 Topic 类型
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_Goal

class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        
        # 订阅 Action 的 Goal 话题（ROS2 Action 底层实现）
        self.subscription = self.create_subscription(
            FollowJointTrajectory_Goal,
            '/main_controller/follow_joint_trajectory/_action/goal',
            self.listener_callback,
            10)
        self.get_logger().info('正在等待 MoveIt 执行指令...')

    def listener_callback(self, msg):
        # msg.goal 是真正的 FollowJointTrajectory_Goal
        points = msg.goal.trajectory.points
        self.get_logger().info(f'收到新轨迹！包含点数: {len(points)}')
        
        print("-" * 50)
        print(f"{'序号':<5} | {'时间(s)':<8} | {'关节角度 (Degrees)':<40}")
        
        for i, point in enumerate(points):
            # 将弧度转换为角度
            degrees = [round(math.degrees(rad), 2) for rad in point.positions]
            
            # 格式化时间
            time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            print(f"{i:<5} | {time_sec:<8.3f} | {degrees}")
        print("-" * 50)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()