import py_trees
from actions import DetectSurvivor
from zone_manager import ZoneManager


class ZonesRemaining(py_trees.behaviour.Behaviour):
    def __init__(self, name, zone_manager: ZoneManager):
        pass


class IsSurvivorDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name, detect_node: DetectSurvivor):
        super(IsSurvivorDetected, self).__init__(name)
        # Hold a reference to the DetectSurvivor action node.
        self.detect_node = detect_node

    def update(self):
        if self.detect_node.was_found():
            self.logger.info(f"Condition {self.name}: Survivor was detected.")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info(f"Condition {self.name}: No survivor detected.")
            return py_trees.common.Status.FAILURE
