#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory
import math, time
from scservo_sdk import *     # Official SCServo SDK


class SO100Controller(Node):
    def __init__(self):
        super().__init__('so100_controller')

        # === SERIAL CONFIGURATION ===
        self.port_name = '/dev/ttyACM0'   # change if using /dev/ttyACM0
        self.baud_rate = 1000000
        self.portHandler = PortHandler(self.port_name)
        self.packetHandler = sms_sts(self.portHandler)

        # Open and configure port
        if not self.portHandler.openPort():
            self.get_logger().error(f"Cannot open {self.port_name}")
            quit()
        self.portHandler.setBaudRate(self.baud_rate)
        self.get_logger().info(f"Connected to {self.port_name} @ {self.baud_rate}")

        # === SERVO CONFIG ===
        self.joint_names = ['Rotation','Pitch','Elbow','Wrist_Pitch','Wrist_Roll','Jaw']
        self.prev_positions = [0.0]*6

        # === ROS INTERFACE ===
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe to MoveIt’s DisplayTrajectory topic
        self.sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.display_traj_callback,
            10
        )

        # 20 Hz feedback loop
        self.timer = self.create_timer(0.05, self.feedback_loop)

        self.get_logger().info("🚀 SO100 Controller ready — listening on /display_planned_path")

    # -----------------------------------------------------------
    # Conversions
    # -----------------------------------------------------------
    def rad2tick(self, rad): return int(2048 + (rad / math.pi) * 2048)
    def tick2rad(self, tick): return (tick - 2048) / 2048.0 * math.pi

    # -----------------------------------------------------------
    # Feedback publisher
    # -----------------------------------------------------------
    def feedback_loop(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        positions = []

        for i in range(1,7):
            try:
                pos, result, error = self.packetHandler.ReadPos(i)
                if result == COMM_SUCCESS and error == 0:
                    rad = self.tick2rad(pos)
                    positions.append(rad)
                else:
                    positions.append(self.prev_positions[i-1])
            except Exception:
                positions.append(self.prev_positions[i-1])

        msg.position = positions
        self.prev_positions = positions
        self.joint_pub.publish(msg)

    # -----------------------------------------------------------
    # DisplayTrajectory callback (from MoveIt)
    # -----------------------------------------------------------
    def display_traj_callback(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            self.get_logger().warn("DisplayTrajectory is empty")
            return
        traj = msg.trajectory[0].joint_trajectory
        self.execute_trajectory(traj)

    # -----------------------------------------------------------
    # Execute the embedded JointTrajectory
    # -----------------------------------------------------------
    def execute_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Trajectory has no points!")
            return

        joint_map = [self.joint_names.index(n) for n in msg.joint_names]
        self.get_logger().info(f"📡 Executing trajectory with {len(msg.points)} waypoints")

        start = self.get_clock().now()
        for idx, pt in enumerate(msg.points):
            dur = pt.time_from_start.sec + pt.time_from_start.nanosec/1e9
            self.get_logger().info(f"→ Waypoint {idx+1}/{len(msg.points)} @ {dur:.2f}s")

            for j, rad in enumerate(pt.positions):
                sid = joint_map[j] + 1
                ticks = self.rad2tick(rad)
                comm, err = self.packetHandler.WritePosEx(sid, ticks, 800, 200)
                if comm != COMM_SUCCESS:
                    self.get_logger().warn(f"Servo {sid}: {self.packetHandler.getTxRxResult(comm)}")
                elif err != 0:
                    self.get_logger().warn(f"Servo {sid}: {self.packetHandler.getRxPacketError(err)}")

            elapsed = (self.get_clock().now() - start).nanoseconds/1e9
            time.sleep(max(0.0, dur - elapsed))

        self.get_logger().info("Trajectory execution complete")

    # -----------------------------------------------------------
    def destroy_node(self):
        self.portHandler.closePort()
        super().destroy_node()


# -----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SO100Controller()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down controller")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

