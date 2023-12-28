import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.time_to_move = 7.0  # Initial movement duration
        self.i = 0

    def timer_callback(self):
        msg = Twist()

        # Linear velocity
        if self.i < self.timer_period / 2 / 0.1:
            msg.linear.x = 0.1
        else:
            msg.linear.x = 0.0

        # Angular velocity for rotation
        if self.i >= self.time_to_move / 0.1:
            msg.angular.z = 0.0
        else:
            msg.angular.z = math.radians(90) / self.time_to_move

        self.publisher_.publish(msg)
        self.i += 1

        # Change movement after specified duration
        if self.i >= self.time_to_move / 0.1:
            self.i = 0
            if self.time_to_move == 7.0:
                self.time_to_move = 1.5708  # 90 degrees in radians
            elif self.time_to_move == 1.5708:
                self.time_to_move = 3.5
            elif self.time_to_move == 3.5:
                self.time_to_move = 1.5708
            elif self.time_to_move == 1.5708:
                self.time_to_move = 7.0

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
