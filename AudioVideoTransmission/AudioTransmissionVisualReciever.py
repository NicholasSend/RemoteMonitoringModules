from time import sleep

from vidstream import AudioSender, AudioReceiver, StreamingServer
import threading

# To be used on the robot reception end of the system

# Initialize transmitter/receiver objects
server = StreamingServer('10.147.20.55', 9999)
receiver = AudioReceiver('10.147.20.55', 4000)
sender = AudioSender('10.147.20.101', 5000)

# Initialize the threads
receive_thread = threading.Thread(target=receiver.start_server)
send_thread = threading.Thread(target=sender.start_stream)
server_thread = threading.Thread(target=server.start_server)

# Start the Threads
server_thread.start()
receive_thread.start()
sleep(15)  # Robot requires time to process prior to message send
send_thread.start()
