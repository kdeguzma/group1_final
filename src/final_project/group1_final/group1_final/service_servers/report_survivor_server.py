# ENPM605 - RO01
# Group Final Project - Group 1
# Kyle DeGuzman: 120452062
# Stephen Snelson: 12254074
# report_survivor_server.py - acknowledge survivor locations and simulate a command center.

import rclpy
from group1_final_interfaces.srv import ReportSurvivor
from rclpy.node import Node


class ReportSurvivorServer(Node):
    """Service Server that acknowledges reported survivor locations"""

    def __init__(self, node_name: str = "report_survivor_server") -> None:
        """Initialize the rport servivor server node.

        args:
            node_name: Name assigned
        """
        super().__init__(node_name)

        self._service = self.create_service(
            ReportSurvivor,
            "report_survivor",
            self._handle_report,
        )

        self.get_logger().info("Report Survivor Service Ready")

    def _handle_report(
        self, request: ReportSurvivor.Request, response: ReportSurvivor.Response
    ) -> ReportSurvivor.Response:
        """Handle a survivor report request
        args:
            request: Report request containing survivor information
            response: Response object to populate acknowledgement
        returns:
            ReportSurvivor.Response: The populated Report Survivor Response
        """
        
        #unpack request information
        frame_id = request.location.header.frame_id
        x = request.location.point.x
        y = request.location.point.y

        if frame_id != "map":
            self.get_logger().warn(
                f"Report for {request.survivor_id} used from frame '{frame_id}expected from 'map'"
            )

        self.get_logger().info(
            f"Report received: {request.survivor_id} at ({x:.2f},{y:.2f} in frame {frame_id}. Acknowledged)"
        )

        response.acknowledged = True
        return response


# Temporary main function for debugging.


def main(args=None):
    rclpy.init(args=args)
    node = ReportSurvivorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
