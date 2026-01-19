#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperMirrorController(Node):
    def __init__(self):
        super().__init__('gripper_mirror_controller')

        # subscribe to the controller state that contains left/right joint7 names
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/controller_state',
            self.joint_state_callback,
            10
        )

        # single publisher (message will contain left/right joint8 as joint_names)
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/gripper8_controller/joint_trajectory',
            10
        )

        # publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_joint8_command)

        # store observed positions for left/right joint7
        self.joint7_positions = {}

        # mappings from input joint to mirrored joint name
        self.mirror_map = {
            'left_joint7': 'left_joint8',
            'right_joint7': 'right_joint8'
        }

    def joint_state_callback(self, msg):
        # controller state message may have 'desired', 'actual', or older 'reference'
        candidates = []
        for attr in ('desired', 'actual', 'reference'):
            part = getattr(msg, attr, None)
            if part is not None:
                candidates.append(part)

        if not candidates:
            self.get_logger().warning('No desired/actual/reference in controller_state')
            return

        # prefer 'actual' if available, otherwise first candidate
        state_point = None
        if getattr(msg, 'actual', None) is not None:
            state_point = msg.actual
        else:
            state_point = candidates[0]

        for in_name, out_name in self.mirror_map.items():
            try:
                idx = msg.joint_names.index(in_name)
                pos = state_point.positions[idx]
                self.joint7_positions[in_name] = pos
            except ValueError:
                # not present in this message - that's fine
                continue

    def publish_joint8_command(self):
        # for each observed joint7 (left/right), publish a mirrored joint8 command
        for in_name, pos in list(self.joint7_positions.items()):
            out_name = self.mirror_map.get(in_name)
            if out_name is None:
                continue

            joint8_position = -pos

            traj_msg = JointTrajectory()
            traj_msg.joint_names = [out_name]

            point = JointTrajectoryPoint()
            point.positions = [joint8_position]

            traj_msg.points = [point]

            self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperMirrorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
