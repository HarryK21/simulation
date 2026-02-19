#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import os
import random
import math
from scipy.spatial.transform import Rotation as R

class AprilTagSpawner(Node):
    def __init__(self):
        super().__init__('apriltag_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn_entity service not available, waiting...')

        self.spawn_random_tags()

    def spawn_random_tags(self):
        z_offset = 0.3  # height above ground (e.g., pallet height)

        # Define 3 spawn locations
        spawn_coords = [
            (6.2, 3.0),
            (6.2, 4.0),
            (6.2, 5.0)
        ]

        # Available Apriltag models (you said they exist with these names)
        available_tags = [f'tag41_12_0000{i}' for i in range(1, 10)]

        model_path = os.path.join(get_package_share_directory('tb3_gazebo'), 'models')

        chosen_tags = random.sample(available_tags, 3)

        for i, (x, y) in enumerate(spawn_coords):
            tag_model = chosen_tags[i]
            sdf_path = os.path.join(model_path, tag_model, 'model.sdf')

            with open(sdf_path, 'r') as f:
                sdf_xml = f.read()

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z_offset)

            # Orientation
            q = R.from_euler('xyz', [-math.pi/2, 0.0, math.pi/2]).as_quat()
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            req = SpawnEntity.Request()
            req.name = f'{tag_model}_{i}'  # unique name
            req.xml = sdf_xml
            req.robot_namespace = ''
            req.reference_frame = 'world'
            req.initial_pose = pose

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info(f'Successfully spawned {req.name} at ({x}, {y})')
            else:
                self.get_logger().error(f'Failed to spawn {req.name}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
