import math

import py_trees
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped

# Import the .srv files.
from group1_final_interfaces import DetectSurvivor, ReportSurvivor
from nav2_msgs.action import NavigateToPose
from py_trees.common import Status
from rclpy.action import ActionClient
from rclpy.tasks import Future

# Import the ZoneManager class from zone_manager.py
from zone_manager import ZoneManager


class NavigateToZone(py_trees.behaviour.Behaviour):
    """Class to send a NavigateToPose goal to Nav2 for the
    current zone from zone_manager."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): An instance of the ZoneManager class.
        """
        # Initialize the node with the name.
        super().__init__(name)
        # Store ZoneManager's instance as an attribute.
        self.zone_manager = zone_manager
        # Store the node, client, goal handle, status, and future goal as internal attributes.
        self.node = None
        self.client = None
        self.goal_handle = None
        self._status = None
        self._send_goal_future = None

    def setup(self, **kwargs) -> None:
        """Function to extract the ROS node from the BT setup kwargs.

        Args:
            **kwargs: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self.node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self.node:
            raise RuntimeError("No ROS node provided to NavigateToZone")
        # Create a client by calling ActionClient and NavigateToPose.
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def initialize(self) -> None:
        """Function to reset the state and prepare the goal."""
        # Reset the goal handle and status.
        self.goal_handle = None
        self._status = None

    def update(self) -> Status:
        """Function to update the NavigateToPose goal for Nav2.

        Returns:
            Status: Set as RUNNING since the goal is being updated.
        """
        # If the goal handle is None and the status is None, log a message
        # to wait for the server if not done already, and return RUNNING for the status.
        if self.goal_handle is None and self._status is None:
            if not self.client.wait_for_server(timeout_sec=1.0):
                self.logger.info("Waiting for Nav2 action server...")
                return Status.RUNNING
            # Access the (id, x, y, yaw) dict from ZoneManager's current_zone function.
            zone = self.zone_manager.current_zone()

            # Set a goal message for NavigateToPose. Set the ID, time stamp, position, and orientation.
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            # Get the x and y values from the dict and set the position.
            goal_msg.pose.pose.position.x = float(zone["x"])
            goal_msg.pose.pose.position.y = float(zone["y"])

            # Get the yaw value from the dict and set the orientation.
            # Convert the yaw to a quaternion.
            yaw = zone["yaw"]
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            # Use an Asynchronous call to send the goal message to the client. Then call
            # _goal_response_callback and return RUNNING as the status.
            self._send_goal_future = self.client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self._goal_response_callback)

            return Status.RUNNING

        # If the status is still None, set it as RUNNING.
        if self._status is None:
            return Status.RUNNING

        return self._status

    def _goal_response_callback(self, future: Future) -> None:
        """Function for the goal response callback.

        Args:
            future (Future): The future object containing the goal handle.
        """
        # Store the result of the future object as an internal attribute goal_handle.
        self.goal_handle = future.result()
        # If the goal_handle is not accepted, log an error and set the Status as FAILURE,
        # then exit.
        if not self.goal_handle.accepted:
            self.logger.error("Goal rejected by server.")
            self._status = Status.FAILURE
            return
        # Use an Asynchronous call for the goal handle and store it. Then call _get_result_callback.
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future) -> None:
        """Function for the result callback.

        Args:
            future (Future): The future object containing the result status.
        """
        # Get the result status from the future object and store it.
        status = future.result().status
        # If the status matches STATUS_SUCCEEDED, set the _status (the attribute)
        # as SUCCESS. Otherwise, set it as FAILURE.
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._status = Status.SUCCESS
        else:
            self._status = Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Method to cancel navigation if interrupted.

        Args:
            new_status (Status): The new status (needed by py_trees).
        """
        # Use an Asynchronous call if the goal needs to be canceled.
        if self.goal_handle is not None:
            self.logger.info("Canceling navigation goal...")
            self.goal_handle.cancel_goal_async()
        # Reset the goal handle and status.
        self.goal_handle = None
        self._status = None


class NavigateToBase(py_trees.behaviour.Behaviour):
    """Class to send a base pose as a goal for the current
    zone from zone_manager."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): An instance of the ZoneManager class.
        """
        # Initialize the node with the name.
        super().__init__(name)
        # Store ZoneManager's instance as an attribute.
        self.zone_manager = zone_manager
        # Store the node, client, goal handle, status, and future goal as internal attributes.
        self.node = None
        self.client = None
        self.goal_handle = None
        self._status = None
        self._send_goal_future = None

    def setup(self, **kwargs) -> None:
        """Function to extract the ROS node from the BT setup kwargs.

        Args:
            **kwargs: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self.node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self.node:
            raise RuntimeError("No ROS node provided to NavigateToBase")
        # Create a client by calling ActionClient and NavigateToPose.
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def initialize(self) -> None:
        """Reset the state and prepare the goal."""
        # Reset the goal handle and status,
        self.goal_handle = None
        self._status = None

    def update(self) -> Status:
        """Function to update the NavigateToPose goal for Nav2.

        Returns:
            Status: Set as RUNNING since the goal is being updated.
        """
        # If the goal handle is None and the status is None, log a message
        # to wait for the server if not done already, and return RUNNING for the status.
        if self.goal_handle is None and self._status is None:
            if not self.client.wait_for_server(timeout_sec=1.0):
                self.logger.info("Waiting for Nav2 action server...")
                return Status.RUNNING
            # Access the (id, x, y, yaw) dict from ZoneManager's base_pose function.
            base = self.zone_manager.base_pose()

            # Set a goal message for NavigateToPose. Set the ID, time stamp, position, and orientation.
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            # Get the x and y values from the dict and set the position.
            goal_msg.pose.pose.position.x = float(base["x"])
            goal_msg.pose.pose.position.y = float(base["y"])

            # Get the yaw value from the dict and set the position.
            # Convert the yaw to a quaternion.
            yaw = base["yaw"]
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            # Use an Asynchronous call to send the goal message to the client. Then call
            # _goal_response_callback and return RUNNING as the status.
            self._send_goal_future = self.client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self._goal_response_callback)

            return Status.RUNNING

        # If the status is still None, set it as RUNNING.
        if self._status is None:
            return Status.RUNNING

        return self._status

    def _goal_response_callback(self, future: Future) -> None:
        """Function for the goal response callback.

        Args:
            future (Future): The future object containing the goal handle.
        """
        # Store the result of the future object as an internal attribute goal_handle.
        self.goal_handle = future.result()
        # If the goal_handle is not accepted, log an error and set the Status as FAILURE,
        # then exit.
        if not self.goal_handle.accepted:
            self.logger.error("NavigateToBase goal rejected by server.")
            self._status = Status.FAILURE
            return
        # Use an Asynchronous call for the goal handle and store it. Then call _get_result_callback.
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future) -> None:
        """Function for the result callback.

        Args:
            future (Future): The future object containing the result status.
        """
        # Get the result status from the future object and store it.
        status = future.result().status
        # If the status matches STATUS_SUCCEEDED, set the _status (the attribute)
        # as SUCCESS. Otherwise, set it as FAILURE.
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._status = Status.SUCCESS
        else:
            self._status = Status.FAILURE

    def terminate(self, new_status: Status):
        """Method to cancel navigation if interrupted.

        Args:
            new_status (Status): The new status (needed by py_trees).
        """
        # Use an Asynchronous call if the goal needs to be canceled.
        if self.goal_handle is not None:
            self.logger.info("Canceling navigation goal...")
            self.goal_handle.cancel_goal_async()
        # Reset the goal handle and status.
        self.goal_handle = None
        self._status = None


# NOTE: This class had to be renamed from DetectSurvivor to DetectSurvivorAction because it
# shared the same name with the .srv file from group1_final_interfaces. This was necessary to
# prevent it from redefining that and making things confusing.
class DetectSurvivorAction(py_trees.behaviour.Behaviour):
    """Class to call the detect_survivor.srv file with the current_zone ID."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): An instance of the ZoneManager class.
        """
        # Initialize the node with the name.
        super().__init__(name)
        # Store ZoneManager's instance as an attribute.
        self.zone_manager = zone_manager
        # Store the node and client as attributes.
        self.node = None
        self.client = None
        # Store results as attributes.
        self._found = False
        self._pose = (0.0, 0.0)
        self.future = None

    def setup(self, **kwargs) -> None:
        """Function to extract the ROS node from the BT setup kwargs.

        Args:
            **kwargs: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self.node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self.node:
            raise RuntimeError("No ROS node provided to DetectSurvivor.")
        # Create a client by calling the DetectSurvivor.srv file.
        self.client = self.node.create_client(DetectSurvivor, "detect_survivor")

    def update(self) -> Status:
        """Function to start a service call to detect survivors.

        Returns:
            Status: RUNNING if the service call is in progress, SUCCESS if the service
            call is complete, and FAILURE otherwise.
        """
        # Start the service call. If there is no future object and no response from
        # detect_survivor, log the following message and return RUNNING as the status.
        if self.future is None:
            if not self.client.wait_for_service(timeout_sec=1.0):
                self.logger.info("Waiting for detect_survivor service...")
                return Status.RUNNING

            # Store the request from detect_survivor.srv.
            req = DetectSurvivor.Request()
            # Call the detect_survivor.srv with the current zone ID.
            # The function in ZoneManager is current_zone and is a dictionary of (id, x, y, yaw).
            req.zone_id = self.zone_manager.current_zone()["id"]
            # Store an Asynchronous call of the client for req and return RUNNING as the status.
            self.future = self.client.call_async(req)
            return Status.RUNNING

        # Wait for completion, then store the pose and set the found result.
        if self.future.done():
            try:
                # Store the future object's result and store the pose.
                result = self.future.result()
                self._found = result.found
                self._pose = (result.survivor_x, result.survivor_y)
                # Reset the future attribute.
                self.future = None
                # Return SUCCESS when the service call completes.
                return Status.SUCCESS
            # Otherwise, log an error stating the call failed and return
            # FAILURE as the status.
            except Exception as e:
                self.logger.error(f"Service call failed: {e}")
                return Status.FAILURE
        # Return RUNNING as the status otherwise.
        return Status.RUNNING

    # Expose the following methods.

    def was_found(self) -> bool:
        """Method to return if a survivor was found.

        Returns:
            bool: True if a survivor was found.
        """
        return self._found

    def survivor_pose(self) -> tuple[float, float]:
        """Method to return the position of the survivor.

        Returns:
            tuple[float, float]: The coordinates of the survivor's location.
        """
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

        return Status.SUCCESS


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
                return Status.RUNNING
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
            return Status.RUNNING

        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if result.acknowledged:
                    self.logger.info("Base acknowledged the survivor.")
                    return Status.SUCCESS
                else:
                    self.logger.error("Base failed to acknowledge the survivor.")
                    return Status.FAILURE
            except Exception as e:
                self.logger.error(f"NotifyBase service call failed: {e}")
                return Status.FAILURE
        return Status.RUNNING

    def terminate(self):
        self.future = None
        self._sent = False


class AdvanceZone(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        super().__init__(name)
        self.zone_manager = zone_manager

    def update(self) -> Status:
        self.zone_manager.advance()
        self.logger.info(f"{self.name}: Advancing to next zone.")
        return Status.SUCCESS


class LogNoDetection(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
