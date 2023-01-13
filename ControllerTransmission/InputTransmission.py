import socket
import time

from ControllerTransmission.controller_objects.GamepadController import GamepadController


class ControlTransmission(object):
    """
    An object for transmitting formatted input to the ROS Noetic system over UDP

    @author Nicholas Sendyk
    @cite https://wiki.python.org/moin/UdpCommunication
    """

    UDP_IP = "192.168.136.1"
    UDP_PORT = 8888

    def __init__(self):
        """
        Initializes the local objects
        """
        self.game_controller = GamepadController()
        self.bytes_format = 'utf-8'

    def transmit_udp(self):
        """
        Transmits formatted commands from a controller to the ROS noetic system over UDP
        """
        print("UDP target IP: %s" % ControlTransmission.UDP_IP)
        print("UDP target port: %s" % ControlTransmission.UDP_PORT)

        while 1:
            unprocessed_message = self.game_controller.read()

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
        message = ""

        if unprocessed_message[0] > 0.1:
            message += "R" + '{0:.2f}'.format(unprocessed_message[0]) + "|"
        elif unprocessed_message[0] < -0.1:
            message += "L" + '{0:.2f}'.format(abs(unprocessed_message[0])) + "|"
        else:
            message += "N" + '{0:.2f}'.format(abs(unprocessed_message[0])) + "|"

        if unprocessed_message[1] > 0.1:
            message += "F" + '{0:.2f}'.format(unprocessed_message[1]) + "|"
        elif unprocessed_message[1] < -0.1:
            message += "B" + '{0:.2f}'.format(abs(unprocessed_message[1])) + "|"
        else:
            message += "N" + '{0:.2f}'.format(abs(unprocessed_message[1])) + "|"

        msg = bytes(message, self.bytes_format)

        return msg


if __name__ == '__main__':
    ControlTransmission().transmit_udp()
