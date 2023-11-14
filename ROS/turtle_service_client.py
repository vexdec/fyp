# Import necessary modules
from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node

# Define a class for the TurtleColorClient, which is a subclass of rclpy's Node class
class TurtleColorClient(Node):
    def __init__(self):
        # Call the constructor of the base class (Node) with the node name "turtle_color_client"
        super().__init__("turtle_color_client")

        # Create a client to interact with the "/spawn" service from turtlesim
        self.cli = self.create_client(Spawn, "/spawn")

        # Wait for the service to become available, with a timeout of 1 second
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    # Define a method to send a request to the service to spawn a turtle
    def send_request(self, x, y, theta):
        # Create a request object of the type "Spawn.Request"
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta

        # Call the service asynchronously and store the future object for the response
        self.future = self.cli.call_async(req)

        # Spin the node until the future is complete (response received or timeout)
        rclpy.spin_until_future_complete(self, self.future)

        # Return the result of the future (response from the service)
        return self.future.result()

class SetPenRequest(Node):
    def __init__(self):
        super().__init__("turtle_colour_client_")
        
        self.cli2 = self.create_client(SetPenRequest, "turtle2/set_pen")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.getlogger().info("Service not available, waiting again...")

    def 
        
# Define the main function
def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create an instance of the TurtleColorClient
    color_client = TurtleColorClient()

    # Send a request to spawn a turtle at position (1.5, 1.0) with an orientation of 0.0
    response = color_client.send_request(1.5, 1.0, 0.0)

    # Print the name of the spawned turtle obtained from the service response
    color_client.get_logger().info(f"Turtle spawned: {response.name}")

    set_pen_request = colour_client.SetPenRequest(0, 255, 0)

    # Clean up and shut down the ROS client library
    color_client.destroy_node()
    rclpy.shutdown()

# Entry point of the script, check if it's the main module and call the main function
if __name__ == "__main__":
    main()


