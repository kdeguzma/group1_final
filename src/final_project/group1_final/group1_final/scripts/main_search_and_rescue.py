import rclpy
from service_servers.detect_survivor_server import DetectSurvivorServer


def SearchAndRescue(args=None):
    rclpy.init(args=args)
    node = DetectSurvivorServer("detect_survivor_server")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
