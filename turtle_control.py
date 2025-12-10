import rclpy
from rclpy.node import Node
from geommetry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleControlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.subscriber_ = self.create_subscription(Pose,'/turtle1/pose',self.pose_receive,10)
        self.target_x_ = 1.0
        self.target_y_ = 1.0
        self.k_ = 1.0
        self.max_speed_ = 3.0

    def pose_receive(self, pose):
        # 1.当前速度
        current_x = pose.x
        current_y = pose.y
        print(f"当前速度X:{current_x},y:{current_y}")

        # 2.计算角速度
        distance = sqrt(
            (self.target_x_ - current_x) * (self.target_x_ - current_x) +
            (self.target_y_ - current_y) * (self.target_y_ - current_y)
        )

        angle = atan2((self.target_y_ - current_y),(self.target_x_ - current_x)) - pose.theta # 计算当前角度差

        # 3.控制策略
        msg = Twist()
        if distance > 0.1:
            if fabs(angle) > 0.2:
                msg.angular.z = fabs(angle)
            else:
                msg.linear.x = self.k_ * distance
        
        # 4.限制限速度最大值
        if msg.linear.x > self.max_speed_:
            msg.linear.x = self.max_speed_

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = TurtleControlNode('turtle_control')
    node.get_logger().info('启动接受')
    rclpy.spin(node)
    rclpy.shutdown()

