import math
import rclpy
# Import necessary modules
from rclpy.action import ActionServer
from rclpy.action.server import (
    ServerGoalHandle,
    GoalResponse,
    CancelResponse,
    GoalStatus,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
# Import specific message types
from nav2_msgs.action import NavigateToPose
from turtlesim.msg import Pose

import threading

# Custom node to subscribe to sensor data
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("turtle_pose_subscriber")


# Custom node to handle the Turtle action server
class TurtleActionServer(Node):
    def __init__(self):
        super().__init__("turtle_action_server")
        # Initialize variables
        self._goal_handle = None
        self._thead_lock = threading.Lock()
        # Create an Action Server to handle "NavigateToPose" actions
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "turtle_to_pose",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )
        # Create a publisher to send velocity commands to the turtle
        self.get_logger().info("Turtle action server started")
        self._publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # Create a subscriber to get the turtle's current pose
        self._subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.listener_callback, 10
        )
        self._subscription  # prevent unused variable warning

    # Callback to receive the turtle's current pose
    def listener_callback(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    # Goal callback for accepting new goals
    def goal_callback(self, goal_request):
        self.get_logger().info("Goal request received")
        return GoalResponse.ACCEPT

    # Cancel callback to handle goal cancellations
    def cancel_callback(self, goal):
        self.get_logger().info("Cancel request received")
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info("Canceling previous goal")
        return CancelResponse.ACCEPT

    # Callback when a new goal is accepted for execution
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self._thead_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    # Callback to execute the goal (navigate to a pose)
    def execute_callback(self, goal_handle: ServerGoalHandle):
        request: NavigateToPose.Goal = goal_handle.request
        feedback_msg = NavigateToPose.Feedback()

        # Get the target coordinates from the goal
        target_x = request.pose.pose.position.x
        target_y = request.pose.pose.position.y

        self.get_logger().info("Goal Executing")
        self.get_logger().info(str(target_x) + " " + str(target_y))

        # Set parameters for the motion control
        rate = self.create_rate(20)
        distanceTolerance = 0.05
        angleTolerance = 0.05
        linearSpeed = 0.5
        angularSpeed = 0.2
        distanceReached = False
        angleReached = False

        # Perform the motion control until the goal is reached
        while not (distanceReached and angleReached) and rclpy.ok():
            # Calculate distance from current position to the target
            distance = ((target_x - self.x) ** 2 + (target_y - self.y) ** 2) ** 0.5

            if distance < distanceTolerance:
                distanceReached = True
                # If distance is reached, ignore angle and trigger success
                self.get_logger().info("Goal reached")
                goal_handle.succeed()
                break

            # Calculate angle between target and current turtle pose
            dx = target_x - self.x
            dy = target_y - self.y
            angle = math.atan2(dy, dx)

            # Create a velocity command message
            msg = Twist()

            if abs(self.theta - angle) > angleTolerance:
                # If the angle is not within tolerance, adjust the turtle's orientation
                msg.angular.z = (
                    -angularSpeed if self.theta - angle > 0 else angularSpeed
                )

            if (
                distance > distanceTolerance
                and abs(self.theta - angle) < 2 * angleTolerance
            ):
                # If the turtle is facing roughly towards the target, move forward
                msg.linear.x = linearSpeed

            # Send the velocity command to the turtle
            self._publisher.publish(msg)

            # Update the feedback message with current turtle pose and remaining distance
            feedback_msg.current_pose.pose.position.x = self.x
            feedback_msg.current_pose.pose.position.y = self.y
            feedback_msg.current_pose.pose.orientation.z = self.theta
            feedback_msg.distance_remaining = distance

            # Publish feedback to update the action client
            goal_handle.publish_feedback(feedback_msg)

            # Handle goal cancellation
            if self._goal_handle.status == GoalStatus.STATUS_CANCELING:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()
                break

            rate.sleep()

        # Create an empty result and return it to the action client
        result = NavigateToPose.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    # Create the TurtleActionServer node
    action_server = TurtleActionServer()
    # Create a MultiThreadedExecutor to allow concurrent action handling
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(action_server)
    # Start the ROS spin loop for the action server
    rclpy.spin(action_server, executor=multi_thread_executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


