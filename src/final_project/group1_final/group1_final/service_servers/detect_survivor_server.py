# Import the .srv file.
from group1_final_interfaces import DetectSurvivor
from rclpy.node import Node

# Import the main file.
# from scripts.main_search_and_rescue import SearchAndRescue


class DetectSurvivorServer(Node):
    def __init__(self, node_name: str) -> None:
        # Initialize the node using node_name from the main file.
        super().__init__(node_name)
        # Maintain a hardcoded dict mapping zone IDs to survivor locations.
        self.SURVIVORS = {
            "zone_a": (-2.5, 3.2),
            "zone_c": (4.1, -2.5),
        }
        self.srv = self.create_service(
            DetectSurvivor,  # Advertise the service from the .srv file.
            "detect_survivor",  # The service name to be used for the client.
            self.detect_survivor_callback,  # Callback function defined later.
        )
        self.get_logger().info("DetectSurvivor service ready.")

    def detect_survivor_callback(self, request, response):
        # Call the (string) zone_id from DetectSurvivor.srv.
        zone_id = request.zone_id
        # If the zone_id is in the dict, returns found=True with the survivor's (x, y)
        # coordinates. Log the appropriate message.
        if zone_id in self.SURVIVORS:
            x, y = self.SURVIVORS[zone_id]
            response.found = True
            response.x = x
            response.y = y
            self.get_logger().info(
                f"Detection request for {zone_id}: FOUND at ({response.x, response.y})"
            )
        # Otherwise, return False and log the appropriate message.
        else:
            response.found = False
            self.get_logger().info(f"Detection request for {zone_id}: NOT FOUND")
        return response
