import py_trees
from .actions import DetectSurvivorAction
from zone_manager import ZoneManager


class ZonesRemaining(py_trees.behaviour.Behaviour):
    '''Class to detect if there are any remaining zones.'''
    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initializer function for this class.

        Args:
            name (str): The name of the node.
            zone_manager (ZoneManager): The ZoneManager class from zone_manager.py.
        """
        # Set the node name.
        super().__init__(name)
        # Store the shared mission state.
        self.zone_manager = zone_manager

    def update(self) -> py_trees.common.Status
        '''Function to check the remaining state for the zone_manager.
        
        Returns:
            py_trees.common.Status: SUCCESS if there are remaining zones.
        '''
        # If there are remaining zones, return SUCCESS. self.logger() and not self.get_logger()
        # must be used since these are BT nodes!
        if self.zone_manager.has_remaining():
            self.logger.debug("Zones remaining: SUCCESS")
            return py_trees.common.Status.SUCCESS
        # Otherwise, return FAILURE. 
        else:
            self.logger.debug("Zones remaining: FAILURE")
            return py_trees.common.Status.FAILURE


class IsSurvivorDetected(py_trees.behaviour.Behaviour):
    '''Class to detect if there are any survivors.'''
    def __init__(self, name: str, detect_node: DetectSurvivorAction) -> None:
        '''Initializer function for this class.
        
        Args:
            name (str): The name of the node.
            detect_node (DetectSurvivor): The action node from the actions.py file.
        '''
        # Set the node name.
        super().__init__(name)
        # Hold a reference to the DetectSurvivor action node.
        self.detect_node = detect_node

    def update(self) -> py_trees.common.Status:
        '''Function to check if a survivor was found based on DetectSurvivorAction.
        
        Returns:
            py_trees.common.Status: SUCCESS if a survivor was detected, FAILURE
            otherwise.
        '''
        # If a survivor was found, return SUCCESS.
        if self.detect_node.was_found():
            self.logger.info(f"{self.name}: Survivor was detected.")
            return py_trees.common.Status.SUCCESS
        # Otherwise, return FAILURE.
        else:
            self.logger.info(f"{self.name}: No survivor detected.")
            return py_trees.common.Status.FAILURE
