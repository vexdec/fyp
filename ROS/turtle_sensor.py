import rclpy  # Import the ROS 2 Python client library

from rclpy.node import Node  # Import the Node class from rclpy
from turtlesim.msg import Color  # Import the Color message type from turtlesim.msg


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("sensor_subscriber")  # Call the constructor of the Node class
        self.subscription = self.create_subscription(
            Color, "/turtle1/color_sensor", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Color):
        # Log the received color values
        self.get_logger().info(f"Color: r: {msg.r}, g: {msg.g}, b: {msg.b}")


# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    speed_publisher = (
        SensorSubscriber()
    )  # Create an instance of the SensorSubscriber class
    rclpy.spin(speed_publisher)  # Spin the node until it is shut down
    speed_publisher.destroy_node()  # Explicitly destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library


if __name__ == "__main__":
    main()  # Call the main function when the script is executed directly

