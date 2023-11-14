# Import necessary modules
import rclpy
import math
import random
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.action import RotateAbsolute


# Define a class for the TurtleRotateActionClient, which is a subclass of rclpy's Node class
class TurtleRotateActionClient(Node):
    def __init__(self):
        # Call the constructor of the base class (Node) with the node name "turtle_rotate_client"
        super().__init__("turtle_rotate_client")

        # Create an ActionClient to interact with the "/turtle1/rotate_absolute" action server
        self._action_client = ActionClient(
            self, RotateAbsolute, "/turtle1/rotate_absolute"
        )

    # Define a method to send a rotation goal to the action server
    def send_goal(self, theta):
        # Log a message indicating the goal (rotation angle) received
        self.get_logger().info(f"Goal received: {theta}")

        # Create a goal message of type RotateAbsolute.Goal() and set the rotation angle
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = theta

        # Wait for the action server to become available
        self._action_client.wait_for_server()

        # Send the goal message to the action server asynchronously and attach a feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # Attach a callback to handle the response from the action server
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Define a callback method to handle the action server's response to the goal
    def goal_response_callback(self, future):
        # Get the goal handle from the future result
        goal_handle = future.result()

        # Check if the goal was accepted by the action server
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        # If the goal was accepted, log a message indicating that the goal was accepted
        self.get_logger().info("Goal accepted")

        # Get the future result to get the final result of the action
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Define a callback method to handle the final result of the action
    def get_result_callback(self, future):
        # Get the result from the future and extract the rotated angle
        result: RotateAbsolute.Result = future.result().result
        self.get_logger().info(f"Angle Rotated: {result.delta}")

        # Shutdown the ROS client library (rclpy)
        rclpy.shutdown()

    # Define a callback method to handle the feedback from the action server
    def feedback_callback(self, msg):
        # Get the feedback from the message and extract the remaining angle to rotate
        feedback: RotateAbsolute.Feedback = msg.feedback
        self.get_logger().info(f"Angle Remaining: {feedback.remaining}")


def main(args=None):
    # Initialize the ROS client library (rclpy)
    rclpy.init(args=args)

    # Create an instance of the TurtleRotateActionClient class
    action_client = TurtleRotateActionClient()

    # Send a rotation goal with a random angle between 0 and 2*pi (full circle)
    action_client.send_goal(random.SystemRandom().uniform(0, 2 * math.pi))

    # Spin the node, allowing it to handle action-related callbacks
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()


