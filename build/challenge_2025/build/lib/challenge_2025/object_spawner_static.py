#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os
import time
from gazebo_msgs.srv import SpawnEntity

class ObjectSpawner(Node):
    def __init__(self):
        # spawn entity service must be made available by launching gzclient/gzserver beforehand
        super().__init__('object_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn_entity service not available, waiting...')
        self.spawn_object()

    def spawn_object(self):
        sdf_path = os.path.join(
            get_package_share_directory('tb3_gazebo'),
            'models',
            'cat',
            'model.sdf'
        )

        with open(sdf_path, 'r') as f:
            sdf_xml = f.read()

        pose = Pose()
        pose.position.x = 2.0
        pose.position.y = 4.0
        pose.position.z = 0.2

        req = SpawnEntity.Request()
        req.name = 'cat'
        req.xml = sdf_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose = pose

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned cat')
        else:
            self.get_logger().error('Failed to spawn entity')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
