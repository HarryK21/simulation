#!/usr/bin/env python3

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
number_of_detections = 0

def printer(trans: TransformStamped, tag_name):
    print("----------------------------------------")
    print("Detected Tag: " + tag_name)
    print("Position: ")
    print("  X: " + str(trans.transform.translation.x))
    print("  Y: " + str(trans.transform.translation.y))
    print("  Z: " + str(trans.transform.translation.z))
    print("Orientation: ")
    print("  X: " + str(trans.transform.rotation.x))
    print("  Y: " + str(trans.transform.rotation.y))
    print("  Z: " + str(trans.transform.rotation.z))
    print("  W: " + str(trans.transform.rotation.w))
    print(tag_name)

def timercallback():
    global tfBuffer
    from_frame = "base_link"
    target_tags = ["tag16h5:1", "tag16h5:2", "tag16h5:7", "tag16h5:10", "tag16h5:16", "tag16h5:26"]
    for to_frame in target_tags:
        try:
            # do a lookup transform between 'base_link' and 'marker' frame
            trans = tfBuffer.lookup_transform(from_frame, to_frame, rclpy.duration.Duration())
            # returns TransformStamped() message
            printer(trans, to_frame)
            tfBuffer.clear()
            trans = tfBuffer.lookup_transform(
                from_frame, 
                to_frame, 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except :#Exception as e:
            # exception is raised on extrapolation, 
            # no connection between frames or when frames dont exist
            #print(f"lookup failed : {e}")
            pass 

def main():
    global tfBuffer
    rclpy.init() # init ros client library
    nh = rclpy.create_node('tf2_listener') # create a node with name 'tf2_listener'
    tfBuffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds = 1)) # create a TF2 buffer which saves the TFs for given cache_time
    tf2_ros.TransformListener(tfBuffer, nh) # create TF2 listener which connects buffer with node
    nh.create_timer(0.25, timercallback) # call timercallback every 100ms
    try:
        rclpy.spin(nh) # spin node until exception
    except KeyboardInterrupt:
        nh.destroy_node() # destroy node
        rclpy.shutdown() # shutdown ros client library

if __name__ == '__main__':
    main()