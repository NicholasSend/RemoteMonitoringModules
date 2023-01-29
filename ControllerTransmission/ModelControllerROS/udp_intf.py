import rospy
from std_msgs.msg import String

def udp_pub():
    
    # UDP IP Setup 
    
    IP = "192.168.1.242"
    PORT = 5515

    pub = rospy.Publisher('udp_out', String, queue_size=10)
    rospy.init_node('udp_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # UDP Data Retrieval 
    
        pub_str = data.decode("utf-8")
        rospy.loginfo(pub_str)
        pub.publish(pub_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        udp_pub()
    except rospy.ROSInterruptException:
        pass