#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import os
import random

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn_entity service not available, waiting...')

        self.spawn_all_objects()

    def spawn_all_objects(self):
        # Fixed z offset (pallet height)
        z_offset = 0.15

        # 9 predefined spawn locations
        spawn_locations = [
            (-16.0, 13.0),
            (-14.0, 13.0),
            (-12.0, 13.0),
            (-3.0, 14.0),
            (0.0, 14.0),
            (3.0, 14.0),
            (-16.0, 3.0),
            (-13.0, 3.0),
            (-10.0, 3.0)
        ]

        # Available model names
        model_names = ['postbox', 'construction_cone', 'cabinet']
        model_path = os.path.join(get_package_share_directory('tb3_gazebo'), 'models')

        for i, (x, y) in enumerate(spawn_locations):
            chosen_model = random.choice(model_names)
            sdf_path = os.path.join(model_path, chosen_model, 'model.sdf')

            with open(sdf_path, 'r') as f:
                sdf_xml = f.read()

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z_offset)  # placing on pallet

            req = SpawnEntity.Request()
            req.name = f'{chosen_model}_{i}'  # unique name per object
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
    node = ObjectSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
