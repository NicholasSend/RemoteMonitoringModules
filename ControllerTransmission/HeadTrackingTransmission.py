import socket
import time

from controller_objects.ReverbG2 import ReverbG2


class ControlTransmission(object):
    """
    An object for transmitting formatted input to the ROS Noetic system over UDP

    @author Nicholas Sendyk
    @cite https://wiki.python.org/moin/UdpCommunication
    """

    UDP_IP = "10.147.20.134"
    UDP_PORT = 5565

    def __init__(self):
        """
        Initializes the local objects
        """
        self.headset = ReverbG2()
        self.bytes_format = 'utf-8'

    def transmit_udp(self):
        """
        Transmits formatted commands from a controller to the ROS noetic system over UDP
        """
        print("UDP target IP: %s" % ControlTransmission.UDP_IP)
        print("UDP target port: %s" % ControlTransmission.UDP_PORT)

        while 1:
            unprocessed_message = self.headset.read()

            bytes_msg = self._process_message(unprocessed_message)

            print("message: %s" % bytes_msg)

            sock = socket.socket(socket.AF_INET,  # Internet
                                 socket.SOCK_DGRAM)  # UDP
            sock.sendto(bytes_msg, (ControlTransmission.UDP_IP, ControlTransmission.UDP_PORT))
            time.sleep(0.1)

    def _process_message(self, unprocessed_message: list) -> bytes:
        """
        Logic to process controller commands into desired actions on the ROS system

        :param unprocessed_message: semi-formatted input from the controller
        :return: bytes; the byte formatted message
        """

        left = int(unprocessed_message[0])
        right = int(unprocessed_message[1])
        up = int(unprocessed_message[2])
        down = int(unprocessed_message[3])

        if left == 1:
            message = "L"
        elif right == 1:
            message = "R"
        elif up == 1:
            message = "U"
        elif down == 1:
            message = "D"
        else:
            message = "N"

        msg = bytes(message, self.bytes_format)

        return msg


if __name__ == '__main__':
    ControlTransmission().transmit_udp()
