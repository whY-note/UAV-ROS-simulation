#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

class RefTraj:
    def __init__(self):
        rospy.init_node('ref_traj')
        self.ref_path_publisher=rospy.Publisher('ref_trajectory',Path,queue_size=10)
        
        self.path=Path()
        self.path.header.frame_id = "odom"  # 必须与Rviz中设置的Fixed Frame一致
        
    def run(self):

        rate=rospy.Rate(10)
        x=0.0
        while not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            
            # Todo：换成我们的轨迹
            pose.pose.position.x = x
            pose.pose.position.y = math.sin(x)
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
            
            self.path.poses.append(pose)
            self.path.header.stamp = rospy.Time.now()
            self.real_path_publisher.publish(self.path)
            
            x += 0.1

            rate.sleep()