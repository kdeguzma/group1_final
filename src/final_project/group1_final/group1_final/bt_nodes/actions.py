import py_trees
from group1_final_interfaces import DetectSurvivor
from zone_manager import ZoneManager


class NavigateToZone(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        pass


class NavigateToBase(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        pass


class DetectSurvivor(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        super(DetectSurvivor, self).__init__(name)
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

    def update(self):
        # Start the service call.
        if self.future is None:
            if not self.client.service_is_ready():
                self.logger.info("Waiting for detector_survivor service...")
                return py_trees.common.Status.RUNNING

            req = DetectSurvivor.Request()
            # Call the detect_survivor service with the current zone ID.
            # The function in ZoneManager is current_zone and is a dictionary of (id, x, y, yaw).
            req.zone_id = self.zone_manager.current_zone()["id"]

            self.future = self.client.call_async(req)
            return py_trees.common.Status.RUNNING

        # Wait for completion.
        if self.future.done():
            try:
                result = self.future.result()
                self._found = result.found
                self._pose = (result.x, result.y)

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
    def __init__(self, name, detect_node, zone_manager: ZoneManager):
        pass


class NotifyBase(py_trees.behaviour.Behaviour):
    def __init__(self, name, detect_node):
        pass


class AdvanceZone(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        pass


class LogNoDetection(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        pass
