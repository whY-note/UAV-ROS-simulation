#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

import pandas as pd

class ShowPath:
    def __init__(self):
        rospy.init_node('path_publisher')
        self.real_path_publisher = rospy.Publisher('/trajectory', Path, queue_size=10) # 发布trajectory话题
        # self.odometry_subscriber=rospy.Subscriber('/reed_quad_x/odometry_sensor1/odometry',Odometry,self.odometry_callback ,queue_size=1)

        self.path=Path()
        self.path.header.frame_id = "odom"  # 必须与Rviz中设置的Fixed Frame一致
        
    # def odometry_callback(self,msg):
    #     pose=PoseStamped()
    #     pose.header.stamp = rospy.Time.now()
    #     pose.header.frame_id = "odom"
    #     pose.pose=msg.pose.pose
        
    #     self.path.poses.append(pose)
    #     self.path.header.stamp = rospy.Time.now()
    #     self.real_path_publisher.publish(self.path)
        
    def run(self):
        df_real_traj_fig8=pd.read_excel("src/show_trajectory/src/xk_history_fig8.xlsx",index_col=0)
        t=233
        
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            
            pose.pose.position.x = df_real_traj_fig8.iloc[t,0]
            pose.pose.position.y = df_real_traj_fig8.iloc[t,1]
            pose.pose.position.z = df_real_traj_fig8.iloc[t,2]
            pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
            
            self.path.poses.append(pose)
            self.path.header.stamp = rospy.Time.now()
            self.real_path_publisher.publish(self.path)
            
            t+=1

            rate.sleep()
        
if __name__ == '__main__':
    try:
        showpath = ShowPath()
        showpath.run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass