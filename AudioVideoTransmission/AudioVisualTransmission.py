from vidstream import AudioSender, AudioReceiver, CameraClient
import threading

# To be used on the VR Transmission end of the system

# Initialize transmitter/receiver objects
client1 = CameraClient('10.147.20.55', 9999)
receiver = AudioReceiver('10.147.20.101', 5000)
sender = AudioSender('10.147.20.55', 4000)

# Initialize the threads
receive_thread = threading.Thread(target=receiver.start_server)
send_thread = threading.Thread(target=sender.start_stream)
client_thread = threading.Thread(target=client1.start_stream)

# Start the Threads
receive_thread.start()
send_thread.start()
client_thread.start()
