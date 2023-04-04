import socket
import threading

import pyaudio


def send_audio(stream, sock, other_ip, other_port):
    while True:
        data = stream.read(1024)
        sock.sendto(data, (other_ip, other_port))


def receive_audio(stream, sock):
    while True:
        data, addr = sock.recvfrom(1024)
        stream.write(data)


pa = pyaudio.PyAudio()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 5000))

target_ip = input("Target IP address: ")
target_port = int(input("Target port: "))

send_stream = pa.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=1024)
recv_stream = pa.open(format=pyaudio.paInt16, channels=1, rate=44100, output=True, frames_per_buffer=1024)

send_thread = threading.Thread(target=send_audio, args=(send_stream, sock, target_ip, target_port))
recv_thread = threading.Thread(target=receive_audio, args=(recv_stream, sock))
send_thread.start()
recv_thread.start()
