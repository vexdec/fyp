import rclpy  # Import the ROS 2 Python client library
import math  # Import the math module for mathematical operations

from rclpy.node import Node  # Import the Node class from rclpy
from geometry_msgs.msg import Twist  # Import the Twist message type from geometry_msgs


# Define a SpeedPublisher class that extends the Node class
class SpeedPublisher(Node):
    # Define a list of speed values
    speed = [
        [1.0, 0.0],
        [1.0, 0.0],
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, math.pi / 2],
        [0.0, 0.0],
    ]

    # Define the constructor of the SpeedPublisher class
    def __init__(self):
        super().__init__("speed_publisher")  # Call the constructor of the Node class
        # Create a publisher for the "/turtle1/cmd_vel" topic
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        timer_period = 1  # Set the timer period to 1 second
        # Create a timer with the specified period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speedIndex = 0  # Initialize the speed index to 0

    # Define the timer callback function
    def timer_callback(self):
        msg = Twist()  # Create a new Twist message
        # Set the linear and angular velocity based on the current speed index
        msg.linear.x = self.speed[self.speedIndex][0]
        msg.angular.z = self.speed[self.speedIndex][1]
        # Publish the Twist message to the "/turtle1/cmd_vel" topic
        self.publisher_.publish(msg)
        self.speedIndex += 1  # Increment the speed index for the next iteration
        # Reset the speed index to 0 if it exceeds the maximum index
        if self.speedIndex == len(self.speed):
            self.speedIndex = 0
        # Log the published speed values
        self.get_logger().info(
            f"Publishing speed: x: {msg.linear.x}, z: {msg.angular.z}"
        )


# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    speed_publisher = SpeedPublisher()  # Create an instance of the SpeedPublisher class
    rclpy.spin(speed_publisher)  # Spin the node until it is shut down
    speed_publisher.destroy_node()  # Explicitly destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library


if __name__ == "__main__":
    main()  # Call the main function when the script is executed directly

