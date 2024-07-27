import rclpy
from rclpy.node import Node
import mavros_msgs.msg
import mavros_msgs.srv
from TopicServices import *

class RclHandler(Node):
    def __init__(self, node_name: str, rate: int):
        super().__init__(node_name)
        self.rate = self.create_rate(rate)
        self.connected = False
        self.get_logger().info("Rclpy is up ...")

    def disconnect(self):
        if self.connected:
            self.get_logger().info("shutting down rclpy ...")
            rclpy.shutdown()
            self.connected = False

    def service_caller(self, service: TopicService, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            self.get_logger().info("waiting for ROS service:" + srv)
            client = self.create_client(typ, srv)
            ready = client.wait_for_service(timeout_sec=timeout)
            if not ready:
                self.get_logger().error(f"Service {srv} not available within timeout")
                return None

            self.get_logger().info("ROS service is up:" + srv)
            req = typ.Request()
            

            if typ.__class__.__name__ == "Metaclass_CommandBool":
                self.get_logger().info("CommandBool Girdi")
                req.value = bool(data)  # Assuming the service request has an attribute `value`
            elif typ.__class__.__name__ == "Metaclass_CommandTOL":
                self.get_logger().info("CommandTOL Girdi")
                req.altitude = float(data.altitude)
            elif typ.__class__.__name__ == "Metaclass_SetMode":
                self.get_logger().info("SetMode Girdi")
                req.custom_mode = str(data.custom_mode)
            elif typ.__class__.__name__ == "Metaclass_WaypointPush":
                self.get_logger().info("WaypointPush Girdi")
                req.waypoints = data.waypoints
            elif typ.__class__.__name__ == "Metaclass_WaypointClear":
                self.get_logger().info("WaypointClear Girdi")
                pass
            else:
                req.data = data  # Assuming the service request has an attribute `data`

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                return future.result()
            else:
                self.get_logger().error(f"Service call failed: {future.exception()}")
                return None
        except Exception as e:
            self.get_logger().error(f"Service call error: {e}")
            return None