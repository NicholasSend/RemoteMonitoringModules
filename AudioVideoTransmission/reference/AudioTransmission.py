import threading
from time import sleep

from vidstream import AudioSender, AudioReceiver, CameraClient

client = CameraClient('8.8.8.8', 9999)
receiver = AudioReceiver('8.8.8.8', 5000)
sender = AudioSender('8.8.8.8', 4000)

receive_thread = threading.Thread(target=receiver.start_server)
send_thread = threading.Thread(target=sender.start_stream)
server_thread = threading.Thread(target=client.start_stream)

server_thread.start()
sleep(2)
receive_thread.start()
send_thread.start()
