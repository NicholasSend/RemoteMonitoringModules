import rospy
import socket
from std_msgs.msg import String

def udp_servo():
    
    # UDP IP Setup 
    
    UDP_IP = "10.147.20.134"
    UDP_PORT = 5565

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    pub = rospy.Publisher('servo_in', String, queue_size=10)
    rospy.init_node('udp_servo', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown():
        # UDP Data Retrieval 
        data, addr = sock.recvfrom(1024)
        pub_str = data.decode("utf-8")
        rospy.loginfo(pub_str)
        pub.publish(pub_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        udp_servo()
    except rospy.ROSInterruptException:
        pass
