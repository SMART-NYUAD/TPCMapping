#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage




def timestamp_synchronizer():
    rospy.init_node("timestamp_synchronizer")
    pcl_sub = rospy.Subscriber("/ouster/points", PointCloud2, pcl_callback)
    imu_sub = rospy.Subscriber("/ouster/imu", Imu, imu_callback)
    img_sub = rospy.Subscriber("/thermal/image_raw", Image, img_callback)
    compressed_img_sub = rospy.Subscriber("/thermal/image_raw/compressed", CompressedImage, compressed_img_callback)

    global pcl_pub, imu_pub, img_pub, compressed_img_pub
    pcl_pub = rospy.Publisher("/robot/ouster/points", PointCloud2, queue_size=1)
    imu_pub = rospy.Publisher("/robot/imu/data_raw", Imu, queue_size=1)
    img_pub = rospy.Publisher("/thermal/image_raw_sync", Image, queue_size=1)
    compressed_img_pub = rospy.Publisher("/thermal/image_raw_sync/compressed", CompressedImage, queue_size=1)

    rospy.spin()

def update():
    timestamp = pcl.header.stamp
    # imu.header.stamp = timestamp
    pcl.header.stamp = timestamp
    img.header.stamp = timestamp
    compressed_img.header.stamp = timestamp

    pcl_pub.publish(pcl)
    imu_pub.publish(imu)
    img_pub.publish(img)
    compressed_img_pub.publish(compressed_img)

    # rate = rospy.Rate(15)
    # rate.sleep()

def pcl_callback(msg):
    global pcl
    pcl = msg

    update()

def imu_callback(msg):
    global imu 
    imu = msg

def img_callback(msg):
    global img
    img = msg
        
def compressed_img_callback(msg):
    global compressed_img
    compressed_img = msg

        

if __name__=="__main__":
    timestamp_synchronizer()
