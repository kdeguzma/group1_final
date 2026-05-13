import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import py_trees
import py_trees_ros

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from group1_final.zone_manager import ZoneManager
from group1_final.bt_nodes.actions import NavigateToZone, NavigateToBase, DetectSurvivorAction,AdvanceZone,LogNoDetection,BroadcastSurvivorTF,NotifyBase
from group1_final.bt_nodes.conditions import IsSurvivorDetected, ZonesRemaining

def _seed_amcl_and_wait_for_nav2() -> None:
    """Publish initialpose and block until Nav2 is ACtive."""
    #Seed AMCL with the robot's known spawn pose and wait for Nav2
    navigator = BasicNavigator()

    navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    x = initial_pose.pose.position.x
    y = initial_pose.pose.position.y
    yaw = initial_pose.pose.orientation.w
    
    
    navigator.get_logger().info(f"Seeding AMCL with initial pose ({x:.2f},{y:.2f}, yaw={yaw:.2f})")
    navigator.setInitialPose(initial_pose)

    navigator.get_logger().info("Waiting for Nav2 (AMCL + BT Navigator) to become active...")
    navigator.waitUntilNav2Active()

    navigator.get_logger().info("Nav2 is active.")
    navigator.destroy_node()


def main():
    """Method to complete search and rescue operations"""
    
    # Initialise the ROS 2 client library (creates the global context)
    rclpy.init()

    # Use node to import parameters
    
    node = Node("search_and_rescue",
                automatically_declare_parameters_from_overrides=True,
                allow_undeclared_parameters=True)
    
    # Create zone information data structures to pass to zone_manager
    
    zone_order = (node.get_parameter("zone_order")
                  .get_parameter_value()
                  .string_array_value)
    
    zones = []
    for zone_id in zone_order:
        zones.append(
            {
                "id":zone_id,
                "x":node.get_parameter(f"zones.{zone_id}.x").value,
                "y":node.get_parameter(f"zones.{zone_id}.y").value,
                "yaw":node.get_parameter(f"zones.{zone_id}.yaw").value,
            }
        )
    node.get_logger().info(f"Loaded :{len(zones)} serach zones from parameters")
    
    base_station = {
        "x":node.get_parameter("base_station.x").value,
        "y":node.get_parameter("base_station.y").value,
        "yaw":node.get_parameter("base_station.yaw").value
    }
    
    tick_rate = node.get_parameter("tick_rate_hz").value  
    
    # create zone manager    
    zone_manager =ZoneManager(zones = zones,base_station = base_station)
    
    _seed_amcl_and_wait_for_nav2()
    
    # ------Behavior Tree Setup-------------
    # initial root selector -> if all zones visited, send to NavToBase
    root = py_trees.composites.Selector(name='root', memory=False)
    
    # navigate to base action (done after all zones visited)
    navigate_to_base = NavigateToBase(name='Mission Complete',zone_manager=zone_manager)
    
    # Wrap nav to base in a oneshot decorator so program ends 
    # after reaching base    
    navigate_to_base_oneshot = py_trees.decorators.OneShot(
    name="NavigateToBaseOneShot", child=navigate_to_base,
    policy=py_trees.common.OneShotPolicy.ON_COMPLETION)
    
    # Patrol Sequence Node --Condition: Zone Remaining--> Zones Remaining, NavToZone,DetectSurvivor,HandleDetection,AdvanceZone
    patrol = py_trees.composites.Sequence(name = 'Patrol',memory=True)
    
    # Zones remaining condition --> evaluates if all zones have been visited
    zones_remaining = ZonesRemaining(name="Zones Remaining",zone_manager=zone_manager)
    
    # Navigate to Zone Action -> navigate to unvisited zone
    navigate_to_zone = NavigateToZone(name="Navigate to Zone",zone_manager=zone_manager)
    
    # Detect Survivor Action -> looks for survivors at visited zone
    detect_survivor = DetectSurvivorAction(name="Detect Survivor",zone_manager=zone_manager)
    
    # Handle Detection Selector ---> Sequence: Survivor Found or Log No Detection Action
    handle_detection = py_trees.composites.Selector(name="Handle Detection",memory=False)
    
    # Advance Zone Action -> passes next goal to robot
    advance_zone = AdvanceZone(name="Advance Zone",zone_manager=zone_manager)
    
    # Log No Detection Action -> No survivors detected at this location
    log_no_detection = LogNoDetection(name="Log No Detection", zone_manager=zone_manager)
    
    # Survivor Found Sequence --Condition:IsSurvivorDetector--> BroadcastSurvivorTF, Notify Base    
    survivor_found = py_trees.composites.Sequence(name="survivor found",memory=True)
    
    # IsSurvivorDetected Condition -> True if survivor detected, False otherwise
    is_survivor_detected = IsSurvivorDetected(name="IsSurvivorDetected",detect_node=detect_survivor)
    
    # Broadcast Survivor Action -> alerts if survivor found
    broadcast_survivor_tf = BroadcastSurvivorTF(name="broadcast survivor",detect_node=detect_survivor, zone_manager=zone_manager)
    
    # Notify Base -> Notifies Base of survivor locations
    notify_base = NotifyBase(name="notify_base", detect_node=detect_survivor)
    
    # Add children to Survivor Found Sequence -> Condition:SurvivorDetected, BroadcastSurvivorTF, NotifyBAse
    survivor_found.add_children([is_survivor_detected,broadcast_survivor_tf,notify_base])
    
    # Add Children to Handle Detection Selector-> Sequence:Survivor Found, LogNoDetection   
    handle_detection.add_children([survivor_found,log_no_detection])
    
    # Add children Patrol Sequence -> Condition: ZonesRemainaing, NavToZone,DetectSurvivor,HandleDetection:Selector,AdvZone
    patrol.add_children([zones_remaining, navigate_to_zone, detect_survivor, handle_detection, advance_zone])
    
    # Add children to Root Selector -> Sequence:Patrol, NavtoBase
    root.add_children([patrol,navigate_to_base_oneshot])    
    
    # Wrap the py_trees in the ros py_trees
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(node=node)
    
    # Start periodic timer
    period_ms = int(1000/tick_rate)
    tree.tick_tock(period_ms=period_ms)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()