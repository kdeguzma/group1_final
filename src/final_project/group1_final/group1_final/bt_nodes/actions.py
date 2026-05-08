import math

import py_trees
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, TransformStamped

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
        self._zone_manager = zone_manager
        # Store the node, client, goal handle, status, and sent status as internal attributes.
        self._node = None
        self._client = None
        self._goal_handle = None
        self._status = None
        self._goal_sent: bool = False

    def setup(self, **kwargs) -> None:
        """Function to extract the ROS node from the BT setup kwargs.

        Args:
            **kwargs: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self._node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self._node:
            raise RuntimeError("No ROS node provided to NavigateToZone")
        # Create a client by calling ActionClient and NavigateToPose.
        self._client = ActionClient(self._node, NavigateToPose, "navigate_to_pose")

    def initialise(self) -> None:
        """Function to reset the state and prepare the goal."""
        # Reset the goal sent status, goal handle, and status.
        self._goal_sent = False
        self._goal_handle = None
        self._status = None

    def _send_goal(self, pose_dict: dict[str, float]) -> None:
        """Function to set up the goal message to be sent to the client.

        Args:
            pose_dict (dict[str, float]): The dictionary of ID and pose from the ZoneManager
            current_zone function.
        """
        # Set a goal message for NavigateToPose. Set the ID, time stamp, position, and orientation.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()

        # Get the x and y values from the dict and set the position.
        goal_msg.pose.pose.position.x = float(pose_dict["x"])
        goal_msg.pose.pose.position.y = float(pose_dict["y"])

        # Get the yaw value from the dict and set the orientation.
        # Convert the yaw to a quaternion.
        yaw = float(pose_dict["yaw"])
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Send the goal asynchronously and handle the result in the callback.
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def update(self) -> Status:
        """Function to update the NavigateToPose goal for Nav2.

        Returns:
            Status: Set as RUNNING since the goal is being updated.
        """
        # If the goal handle is None and the status is None, log a message
        # to wait for the server if not done already, and return RUNNING for the status.
        if not self._goal_sent:
            if not self._client.wait_for_server(timeout_sec=1.0):
                self.logger.info("Waiting for Nav2 action server...")
                return Status.RUNNING

            # Call the _send_goal function to send the current_zone goal.
            # Then return RUNNING for the status.
            self._send_goal(self._zone_manager.current_zone())
            self._goal_sent = True
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
        self._goal_handle = future.result()
        # If the goal_handle is not accepted, log an error and set the Status as FAILURE,
        # then exit.
        if not self._goal_handle.accepted:
            self.logger.error("Goal rejected by server.")
            self._status = Status.FAILURE
            return
        # Use an Asynchronous call for the goal handle and store it. Then call _result_callback.
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Future) -> None:
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
        if self._goal_handle is not None and self.status == Status.RUNNING:
            self.logger.info("Canceling navigation goal...")
            self._goal_handle.cancel_goal_async()
        # Reset the goal handle and status.
        self._goal_handle = None
        self._status = None


class NavigateToBase(NavigateToZone):
    """Class to send a base pose as a goal for the current
    zone from zone_manager, reuses NavigateToZone logic."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class."""
        # Initialize the node with the name and zone_manager (from
        # the parent class).
        super().__init__(name, zone_manager)
        # Initialize the mission logged flag.
        self._mission_logged: bool = False

    def update(self) -> Status:
        """Function to update the NavigateToPose goal for Nav2.

        Returns:
            Status: Set as RUNNING since the goal is being updated.
        """
        # If the goal handle is None and the status is None, log a message
        # to wait for the server if not done already, and return RUNNING for the status.
        if not self._goal_sent:
            if not self._client.wait_for_server(timeout_sec=1.0):
                self.logger.info("Waiting for Nav2 action server...")
                return Status.RUNNING
            # Access the (id, x, y, yaw) dict from ZoneManager's base_pose function. Then
            # use it with the parent class's _send_goal function.
            self._send_goal(self._zone_manager.base_pose())
            # Set the goal sent flag as true and return the status as RUNNING.
            self._goal_sent = True
            return Status.RUNNING

        # If the robot is still navigating, return RUNNING as the status.
        if self._status is None:
            return Status.RUNNING

        # Set the status as SUCCESS once the mission is done. Set the
        # _mission_logged flag as True.
        if self._status == Status.SUCCESS and not self._mission_logged:
            self.logger.info("Mission completed.")
            self._mission_logged = True

        return self._status

    def initialise(self) -> None:
        """Function to reset the state."""
        # Reset the parent navigation state.
        super().initialise()
        # Reset the mission logged flag.
        self._mission_logged = False


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
        self._zone_manager = zone_manager
        # Store the node and client as attributes.
        self._node = None
        self._client = None
        # Store results as attributes.
        self._was_found: bool = False
        self._survivor_xy: tuple[float, float] | None = None
        self._future = None
        self._survivor_id = None
        # Store the start time as an attribute.
        self._start_time = None
        self.SERVICE_TIMEOUT_SEC = 5.0

    def setup(self, **kwargs) -> None:
        """Function to extract the ROS node from the BT setup kwargs.

        Args:
            **kwargs: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self._node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self._node:
            raise RuntimeError("No ROS node provided to DetectSurvivor.")
        # Create a client by calling the DetectSurvivor.srv file.
        self._client = self._node.create_client(DetectSurvivor, "detect_survivor")

    def initialise(self) -> None:
        """Function to reset the stored attributes.."""
        self._future = None
        self._was_found = False
        self._survivor_xy = None
        self._survivor_id = None
        self._start_time = None

    def update(self) -> Status:
        """Function to start a service call to detect survivors.

        Returns:
            Status: RUNNING if the service call is in progress, SUCCESS if the service
            call is complete, and FAILURE otherwise.
        """
        # Raise an error if the node is not initialized.
        if self._node is None:
            raise RuntimeError("Node not initialized")
        # Return FAILURE for the status if the time exceeds the 5 s timeout.
        if self._start_time is None:
            self._start_time = self._node.get_clock().now()
        if (
            self._node.get_clock().now() - self._start_time
        ).nanoseconds * 1e-9 > self.SERVICE_TIMEOUT_SEC:
            self.logger.error("detect_survivor timeout")
            self._future = None
            return Status.FAILURE
        # Start the service call. If there is no future object and no response from
        # detect_survivor, log the following message and return RUNNING as the status.
        if self._future is None:
            if not self._client.wait_for_service(timeout_sec=1.0):
                self.logger.info("Waiting for detect_survivor service...")
                return Status.RUNNING

            # Store the request from detect_survivor.srv.
            req = DetectSurvivor.Request()
            # Call the detect_survivor.srv with the current zone ID; this returns if a survivor
            # is present and returns the position.
            # The function in ZoneManager is current_zone and is a dictionary of (id, x, y, yaw).
            req.zone_id = self._zone_manager.current_zone()["id"]
            # Store an Asynchronous call of the client for req and return RUNNING as the status.
            self._future = self._client.call_async(req)
            return Status.RUNNING

        # Wait for completion, then store the pose and set the found result.
        if self._future.done():
            try:
                # Store the future object's result and store the pose.
                result = self._future.result()
                self._was_found = result.found
                self._survivor_xy = (result.survivor_x, result.survivor_y)

                # If a survivor was found, assign the next survivor ID.
                if self._was_found:
                    self._survivor_id = self._zone_manager.next_survivor_id()

                # Reset the future object.
                self._future = None
                # Return SUCCESS when the service call completes.
                return Status.SUCCESS
            # Otherwise, log an error stating the call failed and return
            # FAILURE as the status.
            except Exception as e:
                self.logger.error(f"Service call failed: {e}")
                return Status.FAILURE
        # Return RUNNING as the status otherwise.
        return Status.RUNNING

    def survivor_id(self) -> str | None:
        """Method to allow the survivor_id attribute to be used
        outside of this class (needed for NotifyBase).

        Returns:
            str | None: The survivor ID stored in this class.
        """
        return self._survivor_id

    # Expose the following methods.

    def was_found(self) -> bool:
        """Method to return if a survivor was found.

        Returns:
            bool: True if a survivor was found.
        """
        return self._was_found

    def survivor_pose(self) -> tuple[float, float] | None:
        """Method to return the position of the survivor.

        Returns:
            tuple[float, float] | None: The coordinates of the survivor's location.
        """
        return self._survivor_xy


# class IsSurvivorDetected(name, detect_node): Listed as a condition.


class BroadcastSurvivorTF(py_trees.behaviour.Behaviour):
    """Class to create a StaticTransformBroadcaster and publish
    a built message to TransformStamped."""

    def __init__(
        self, name: str, detect_node: DetectSurvivorAction, zone_manager: ZoneManager
    ) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            detect_node (DetectSurvivorAction): An instance of the DetectSurvivorAction class.
            zone_manager (ZoneManager): An instance of the ZoneManager class.
        """
        # Initialize the node with the name.
        super().__init__(name)
        # Store DetectSurvivorAction's instance as an attribute.
        self._detect_node = detect_node
        # Store ZoneManager's instance as an attribute.
        self._zone_manager = zone_manager
        # Store the node, static transform broadcaster, and child frame ID as attributes.
        self._node = None
        self._broadcaster = None
        self._child_frame_id = None

    def setup(self, **kwargs) -> None:
        """Method to initialize the static transform broadcaster with the ROS node.

        Args:
            kwargs**: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self._node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self._node:
            raise RuntimeError("No ROS node provided to BroadcastSurvivorTF.")
        # Create and store an instance of tf2 StaticTransformBroadcaster for the node.
        self._broadcaster = tf2_ros.StaticTransformBroadcaster(self._node)

    def initialise(self) -> None:
        """Function to reset the survivor ID."""
        self._child_frame_id = None

    def update(self) -> Status:
        """Method to create and publish the TransformStamped message.

        Returns:
            Status: SUCCESS after creating and publishing the message.
        """
        # Set the next survivor ID if there is none.
        self._child_frame_id = self._detect_node.survivor_id()

        # Log an error and return FAILURE for the status if there's no ID.
        if not self._child_frame_id:
            self.logger.error("No survivor ID available.")
            return Status.FAILURE

        # Get the survivor pose.
        pose = self._detect_node.survivor_pose()
        # If there is no pose, log an error and set the status as FAILURE.
        if pose is None:
            self.logger.error("No survivor pose available")
            return Status.FAILURE

        x, y = pose

        # Create an instance of TransformStamped for the message.
        t = TransformStamped()
        # Get the current time stamp and frame ID.
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        # Store the child frame ID.
        t.child_frame_id = self._child_frame_id

        # Get translation with (x, y) from the detect_node results. z is 0 here.
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        # Get the rotation with the identity quaternion. There is no rotation relative to the map.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish the TransformStamped() message instance, then log the ID and coordinates and
        # return SUCCESS as the status.
        self._broadcaster.sendTransform(t)

        self.logger.info(f"Published static TF: {t.child_frame_id} at ({x}, {y})")

        return Status.SUCCESS


class NotifyBase(py_trees.behaviour.Behaviour):
    """Class to call the report_survivor.srv file and update survivor location."""

    def __init__(self, name: str, detect_node: DetectSurvivorAction) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of this node.
            detect_node (DetectSurvivorAction): An instance of the DetectSurvivorAction
            class.
        """
        # Initialize the node with the name.
        super().__init__(name)
        # Store an instance of the DetectSurvivorAction class as an attribute.
        self._detect_node = detect_node
        # Store the node, client, and future as attributes.
        self._node = None
        self._client = None
        self._future = None
        # Initialize the start time.
        self._start_time = None
        # Service timeout of 5 seconds.
        self.SERVICE_TIMEOUT_SEC = 5.0

    def setup(self, **kwargs) -> None:
        """Method to extract the ROS node and create the service client.

        Args:
            **kwargs: Keyword arguments from the BT setup.
        """
        # Get the node from kwargs.
        self._node = kwargs.get("node")
        # Raise an error if there is no node.
        if not self._node:
            raise RuntimeError("No ROS node provided to NotifyBase.")
        # Create a client with ReportSurvivor.
        self._client = self._node.create_client(ReportSurvivor, "report_survivor")

    def initialise(self) -> None:
        """Method to reset the state for a new service call."""
        # Reset the future status and start time.
        self._future = None
        self._start_time = None

    def update(self) -> Status:
        """Function to update the survivor position and status.

        Returns:
            Status: RUNNING if still in progress, SUCCESS if the result was acknowledged,
            and FAILURE otherwise.
        """
        # Raise an error if the node is not initialized.
        if self._node is None:
            raise RuntimeError("Node not initialized")

        # Get the position of the survivor.
        pose = self._detect_node.survivor_pose()
        # Get the survivor ID from the public method defined in DetectSurvivorAction.
        survivor_id = self._detect_node.survivor_id()

        # Log an error and return the status as FAILURE if there is no pose or
        # survivor ID.
        if pose is None or not survivor_id:
            self.logger.error("Missing survivor data.")
            return Status.FAILURE

        # Log a message and start the service request.
        if self._future is None:
            if not self._client.wait_for_service(timeout_sec=1.0):
                self.logger.info("Waiting for report_survivor service...")
                return Status.RUNNING

            # Start the timeout timer.
            if self._start_time is None:
                self._start_time = self._node.get_clock().now()

            # Store the request.
            req = ReportSurvivor.Request()
            # Set the survivor ID in the request.
            req.survivor_id = survivor_id

            # Call PointStamped and store it as a location variable. Get the time stamp
            # and frame ID.
            location = PointStamped()
            location.header.stamp = self._node.get_clock().now().to_msg()
            location.header.frame_id = "map"

            # Set the location pose.
            x, y = pose
            location.point.x = float(x)
            location.point.y = float(y)
            location.point.z = 0.0

            # Set the ReportSurvivor request location with the survivor location.
            req.location = location
            # Use an Asynchronous call to send the request to the client. Then set
            # the sent status as True and return RUNNING for the status.
            self._future = self._client.call_async(req)
            return Status.RUNNING

        # Return FAILURE for the status if the time exceeds the 5 s timeout.
        if self._start_time is not None:
            elapsed = (self._node.get_clock().now() - self._start_time).nanoseconds * 1e-9
            if elapsed > self.SERVICE_TIMEOUT_SEC:
                self.logger.error("report_survivor timeout")
                return Status.FAILURE

        # Handle the response.
        if self._future.done():
            try:
                result = self._future.result()
            # Log an error and return the status as FAILURE if the call fails.
            except Exception as e:
                self.logger.error(f"report_survivor call failed: {e}")
                return Status.FAILURE
            # Raise an error and return the status as FAILURE if report_survivor
            # returns None.
            if result is None:
                self.logger.error("report_survivor returned None")
                return Status.FAILURE

            return Status.SUCCESS if result.acknowledged else Status.FAILURE

        # Otherwise, return RUNNING for the status.
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        """Method to cancel navigation if interrupted.

        Args:
            new_status (Status): The new status (needed by py_trees).
        """
        self._future = None


class AdvanceZone(py_trees.behaviour.Behaviour):
    """Class to use the advance() function from ZoneManager. Advances
    to each zone."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): An instance of the ZoneManager class.
        """
        # Initialize the node with the name.
        super().__init__(name)
        # Store ZoneManager's instance as an attribute.
        self._zone_manager = zone_manager

    def update(self) -> Status:
        """Function to call the advance() function for advancing to each zone.

        Returns:
            Status: SUCCESS if advancing to the next zone.
        """
        # Immediately return SUCCESS if there are no zones left.
        if not self._zone_manager.has_remaining():
            self.logger.info("No remaining zones for advancing.")
            return Status.SUCCESS
        # Call the advance() function from zone_manager.py. Then log a message
        # and return SUCCESS for the status.
        self._zone_manager.advance()
        self.logger.info(
            f"{self.name}: Advancing to next zone index {self._zone_manager.current_index()}"
        )
        return Status.SUCCESS


class LogNoDetection(py_trees.behaviour.Behaviour):
    """Class for when no survivor is detected at the current zone."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): An instance of the ZoneManager class.
        """
        super().__init__(name)
        # Store the node and ZoneManager as attributes.
        self._node = None
        self._zone_manager = zone_manager

    def setup(self, **kwargs) -> None:
        """Function to cache the node.

        Args:
            **kwargs: Must contain "node".
        """
        # Get the node with **kwargs.
        self._node = kwargs.get("node")
        # If there's no node, raise an error.
        if not self._node:
            raise RuntimeError("No node provided to LogNoDetection.")

    def update(self) -> Status:
        """Function to update the status if there is no survivor.

        Returns:
            Status: SUCCESS if no survivor was found.
        """
        # Log the information then return SUCCESS as the status.
        zone_id = self._zone_manager.current_zone()["id"]
        self.logger.info(f"No survivor found in zone {zone_id}.")
        return Status.SUCCESS
