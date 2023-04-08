import rospy
import socket
from std_msgs.msg import String

def udp_pub():
    
    # UDP IP Setup 
    
    UDP_IP = "192.168.105.234"
    UDP_PORT = 5515

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    pub = rospy.Publisher('udp_out', String, queue_size=10)
    rospy.init_node('udp_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # UDP Data Retrieval 
        data, addr = sock.recvfrom(1024)
        pub_str = data.decode("utf-8")
        rospy.loginfo(pub_str)
        pub.publish(pub_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        udp_pub()
    except rospy.ROSInterruptException:
        pass
