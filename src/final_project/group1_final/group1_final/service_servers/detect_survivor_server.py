# ENPM605 - RO01
# Group Final Project - Group 1
# Kyle DeGuzman: 120452062
# Stephen Snelson: 12254074
# detect_survivor_server.py - advertise the service and determine survivor locations.

import rclpy

# Import the .srv file.
from group1_final_interfaces.srv import DetectSurvivor
from rclpy.node import Node


class DetectSurvivorServer(Node):
    """Node class which will response to the detect server action providing Robot if a survivor is at the location"""
    def __init__(self) -> None:
        """Initilaize instance of Detect Survivor Server node"""
        # Initialize the node using node_name from the main file.
        super().__init__("detect_survivor_server")
        # Maintain a hardcoded dict mapping zone IDs to survivor locations.
        self.SURVIVORS = {
            "zone_a": (-2.5, 3.2),
            "zone_c": (4.1, -2.5),
        }
        # Create the service.
        self.srv = self.create_service(
            DetectSurvivor,  # Advertise the service from the .srv file.
            "detect_survivor",  # The service name to be used for the client.
            self.detect_survivor_callback,  # Callback function defined later.
        )
        # Log the following message to the terminal.
        self.get_logger().info("DetectSurvivor service ready.")

    def detect_survivor_callback(self, request, response):
        """Callback to provide information if survivor is in the zone. True if survivor is present"""
        # Call the (string) zone_id from DetectSurvivor.srv.
        zone_id = request.zone_id
        # If the zone_id is in the dict, returns found=True with the survivor's (x, y)
        # coordinates. Log the appropriate message.
        if zone_id in self.SURVIVORS:
            x, y = self.SURVIVORS[zone_id]
            response.found = True
            # Set the response fields from the .srv file.
            response.survivor_x = x
            response.survivor_y = y
            self.get_logger().info(f"Detection request for {zone_id}: FOUND at ({x, y})")
        # Otherwise, return False and log the appropriate message.
        else:
            response.found = False
            self.get_logger().info(f"Detection request for {zone_id}: NOT FOUND")
        return response


# Temporary main function for debugging.


def main(args=None):
    rclpy.init(args=args)
    node = DetectSurvivorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
