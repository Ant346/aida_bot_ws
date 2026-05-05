import rclpy
from rclpy.node import Node


class OdroidDriver(Node):
    def __init__(self):
        super().__init__('odroid_driver')
        self.get_logger().info('odroid_driver node started')


def main(args=None):
    rclpy.init(args=args)
    node = OdroidDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
