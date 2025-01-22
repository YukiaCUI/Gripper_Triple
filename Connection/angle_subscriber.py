import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray

class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        # 创建订阅者，订阅名为 'angles' 的话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'angles',
            self.listener_callback,
            10)
        self.subscription  # 让订阅对象不被垃圾回收

    def listener_callback(self, msg):
        # 打印接收到的角度数据
        self.get_logger().info(f'Received angles: {msg.data}')
        

def main(args=None):
    rclpy.init(args=args)
    angle_subscriber = AngleSubscriber()

    try:
        rclpy.spin(angle_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        angle_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
