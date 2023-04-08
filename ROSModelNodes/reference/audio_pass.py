#!/usr/bin/env python
import threading
from time import sleep

import rospy
from vidstream import AudioSender, AudioReceiver, StreamingServer


# Needed imports here


def audio_trans():
    # Initialization commands here
    # ip = socket.gethostbyname(socket.gethostname())
    # print(ip)
    server = StreamingServer('192.168.225.255', 9999)
    receiver = AudioReceiver('192.168.225.255', 4000)
    sender = AudioSender('192.168.225.55', 5000)

    receive_thread = threading.Thread(target=receiver.start_server)
    send_thread = threading.Thread(target=sender.start_stream)
    server_thread = threading.Thread(target=server.start_server)

    server_thread.start()
    receive_thread.start()
    sleep(2)
    send_thread.start()

    rate = rospy.Rate(100)  # 100hz

    while not rospy.is_shutdown():
        # Looping commands here 

        rate.sleep()


if __name__ == '__main__':
    try:
        audio_trans()
    except rospy.ROSInterruptException:
        pass
