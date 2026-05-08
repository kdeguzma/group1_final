import py_trees
from py_trees.common import Status
from zone_manager import ZoneManager

from .actions import DetectSurvivorAction


class ZonesRemaining(py_trees.behaviour.Behaviour):
    """Class to detect if there are any remaining zones."""

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): The ZoneManager class from zone_manager.py.
        """
        # Set the node name.
        super().__init__(name)
        # Store the shared mission state.
        self._zone_manager = zone_manager

    def update(self) -> Status:
        """Function to check the remaining state for the zone_manager.

        Returns:
            Status: SUCCESS if there are remaining zones.
        """
        # If there are remaining zones, return SUCCESS. self.logger() and not self.get_logger()
        # must be used since these are BT nodes! # Otherwise, return FAILURE.
        return Status.SUCCESS if self._zone_manager.has_remaining() else Status.FAILURE


class IsSurvivorDetected(py_trees.behaviour.Behaviour):
    """Class to detect if there are any survivors."""

    def __init__(self, name: str, detect_node: DetectSurvivorAction) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            detect_node (DetectSurvivor): The action node from the actions.py file.
        """
        # Set the node name.
        super().__init__(name)
        # Hold a reference to the DetectSurvivor action node.
        self._detect_node = detect_node

    def update(self) -> Status:
        """Function to check if a survivor was found based on DetectSurvivorAction.

        Returns:
            Status: SUCCESS if a survivor was detected, FAILURE
            otherwise.
        """
        # If a survivor was found, return SUCCESS. Otherwise, return FAILURE.
        return Status.SUCCESS if self._detect_node.was_found() else Status.FAILURE
