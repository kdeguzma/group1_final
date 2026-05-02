import math

import py_trees
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped

# Import the .srv files.
from group1_final_interfaces import DetectSurvivor, ReportSurvivor
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from zone_manager import ZoneManager


class NavigateToZone(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, zone_manager: ZoneManager):
        super().__init__(name)
        self.zone_manager = zone_manager
        self.node = None
        self.client = None
        self.goal_handle = None
        self._status = None

    def setup(self, **kwargs):
        """Function to extract the ROS node from the BT setup kwargs."""
        self.node = kwargs.get("node")
        if not self.node:
            self.logger.error("No ROS node provided to NavigateToZone")
        # Create a client by calling ActionClient and NavigateToPose.
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def initialize(self):
        """Reset the state and prepare the goal."""
        self.goal_handle = None
        self._status = None

    def update(self):
        if self.goal_handle is None and self.done_status is None:
            if not self.client.server_is_ready():
                self.logger.info("Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING
            # Access the (id, x, y, yaw) dict from ZoneManager's current_zone function.
            zone = self.zone_manager.current_zone()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            # Get the x and y values from the dict.
            goal_msg.pose.pose.position.x = float(zone["x"])
            goal_msg.pose.pose.position.y = float(zone["y"])

            # Convert the yaw to a quaternion.
            yaw = zone["yaw"]
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_goal_future = self.client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self._goal_response_callback)
            self._status = py_trees.common.Status.RUNNING

        return self._status if self._status else py_trees.common.Status.RUNNING

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.logger.error("NavigateToZone goal rejected by server.")
            self._status = py_trees.common.Status.FAILURE
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        status = future.result().status
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self._status = py_trees.common.Status.SUCCESS
        else:
            self._status = py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Method to cancel navigation if interrupted."""
        if self.goal_handle is not None and new_status == py_trees.common.Status.INVALID:
            self.logger.info("Canceling navigation goal...")
            self.goal_handle.cancel_goal_async()

        self.goal_handle = None
        self._status = None


class NavigateToBase(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, zone_manager: ZoneManager):
        super().__init__(name)
        self.zone_manager = zone_manager
        self.node = None
        self.client = None
        self.goal_handle = None
        self._status = None

    def setup(self, **kwargs):
        """Function to extract the ROS node from the BT setup kwargs."""
        self.node = kwargs.get("node")
        if not self.node:
            self.logger.error("No ROS node provided to NavigateToZone")
        # Create a client by calling ActionClient and NavigateToPose.
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def initialize(self):
        """Reset the state and prepare the goal."""
        self.goal_handle = None
        self._status = None

    def update(self):
        if self.goal_handle is None and self.done_status is None:
            if not self.client.server_is_ready():
                self.logger.info("Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING
            # Access the (id, x, y, yaw) dict from ZoneManager's base_pose function.
            base = self.zone_manager.base_pose()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            # Get the x and y values from the dict.
            goal_msg.pose.pose.position.x = float(base["x"])
            goal_msg.pose.pose.position.y = float(base["y"])

            # Convert the yaw to a quaternion.
            yaw = base["yaw"]
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_goal_future = self.client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self._goal_response_callback)
            self._status = py_trees.common.Status.RUNNING

        return self._status if self._status else py_trees.common.Status.RUNNING

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.logger.error("NavigateToBase goal rejected by server.")
            self._status = py_trees.common.Status.FAILURE
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        status = future.result().status
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self._status = py_trees.common.Status.SUCCESS
        else:
            self._status = py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Method to cancel navigation if interrupted."""
        if self.goal_handle is not None and new_status == py_trees.common.Status.INVALID:
            self.logger.info("Canceling navigation goal...")
            self.goal_handle.cancel_goal_async()

        self.goal_handle = None
        self._status = None


# NOTE: This class had to be renamed from DetectSurvivor to DetectSurvivorAction because it
# shared the same name with the .srv file from group1_final_interfaces. This was necessary to
# prevent it from redefining that and making things confusing.
class DetectSurvivorAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        super().__init__(name)
        self.zone_manager = zone_manager
        self.node = None
        self.client = None
        # Store results as attributes.
        self._found = False
        self._pose = (0.0, 0.0)
        self.future = None

    def setup(self, **kwargs):
        """Function to extract the ROS node from the BT setup kwargs."""
        self.node = kwargs.get("node")
        if not self.node:
            self.logger.error("No ROS node provided to DetectSurvivor.")
        # Create a client by calling the DetectSurvivor.srv file.
        self.client = self.node.create_client(DetectSurvivor, "detect_survivor")

    def update(self) -> py_trees.common.Status:
        # Start the service call.
        if self.future is None:
            if not self.client.wait_for_service(timeout_sec=1.0):
                self.logger.info("Waiting for detect_survivor service...")
                return py_trees.common.Status.RUNNING

            req = DetectSurvivor.Request()
            # Call the detect_survivor .srv with the current zone ID.
            # The function in ZoneManager is current_zone and is a dictionary of (id, x, y, yaw).
            req.zone_id = self.zone_manager.current_zone()["id"]

            self.future = self.client.call_async(req)
            return py_trees.common.Status.RUNNING

        # Wait for completion, then store the pose and set the found result.
        if self.future.done():
            try:
                result = self.future.result()
                self._found = result.found
                self._pose = (result.survivor_x, result.survivor_y)

                self.future = None
                # Return SUCCESS when the service call completes.
                return py_trees.common.Status.SUCCESS
            except Exception as e:
                self.logger.error(f"Service call failed: {e}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    # Expose the following methods.

    def was_found(self):
        return self._found

    def survivor_pose(self):
        return self._pose


# class IsSurvivorDetected(name, detect_node): Listed as a condition.


class BroadcastSurvivorTF(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, detect_node, zone_manager: ZoneManager):
        super().__init__(name)
        self.detect_node = detect_node
        self.zone_manager = zone_manager
        self.node = None
        self.static_broadcaster = None

    def setup(self, **kwargs):
        """Method to initialize the static transform broadcaster with the ROS node."""
        self.node = kwargs.get("node")
        if not self.node:
            self.logger.error("No ROS node provided to BroadcastSurvivorTF.")
            return False
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        return True

    def update(self):
        """Method to create and publish the TransformStamped message."""
        t = TransformStamped()

        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "map"

        t.child_frame_id = self.zone_manager.next_survivor_id()
        # Translation with (x, y) from detect_node results.
        pose = self.detect_node.survivor_pose()
        t.transform.translation.x = float(pose[0])
        t.transform.translation.y = float(pose[1])
        t.transform.translation.z = 0.0
        # Rotation with the identity quaternion. There is no rotation relative to the map.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)
        self.logger.info(
            f"Published static TF: {t_child_frame_id} at ({t.transform.translation.x}, {t.transform.translation.y})"
        )

        return py_trees.common.Status.SUCCESS


class NotifyBase(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, detect_node):
        super().__init__(name)
        self.detect_node = detect_node
        self.node = None
        self.client = None
        self.future = None
        self._sent = False

    def setup(self, **kwargs):
        """Method to extract the ROS node and create the service client."""
        self.node = kwargs.get("node")
        if not self.node:
            self.logger.error("No ROS node provided to NotifyBase.")
            return False
        self.client = self.node.create_client(ReportSurvivor, "report_survivor")
        return True

    def initialize(self):
        """Method to reset the state for a new service call."""
        self.future = None
        self._sent = False

    def update(self):
        if self.future is None and not self._sent:
            if not self.client.service_is_ready():
                self.logger.info("Waiting for report_survivor service...")
                return py_trees.common.Status.RUNNING
            req = ReportSurvivor.Request()

            req.survivor_id = "survivor_N"

            location = PointStampe()
            location.header.stamp = self.node.get_clock().now().to_msg()
            location.header.frame_id = "map"

            pose = self.detect_node.survivor_pose()
            location.point.x = float(pose)
            location.point.y = float(pose)
            location.point.z = 0.0

            req.location = location

            self.future = self.client.call_async(req)
            self._sent = True
            return py_trees.common.Status.RUNNING

        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if result.acknowledged:
                    self.logger.info("Base acknowledged the survivor.")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.error("Base failed to acknowledge the survivor.")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.logger.error(f"NotifyBase service call failed: {e}")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

        def terminate(self, new_status):
            self.future = None
            self._sent = False


class AdvanceZone(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        super().__init__(name)


class LogNoDetection(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
