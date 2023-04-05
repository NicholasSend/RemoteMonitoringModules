import socket
import time

from ControllerTransmission.controller_objects.GamepadController import GamepadController
from ControllerTransmission.controller_objects.KeyboardController import KeyboardController
from ControllerTransmission.controller_objects.ReverbG2 import VRController


class ControlTransmission(object):
    """
    An object for transmitting formatted input to the ROS Noetic system over UDP

    @author Nicholas Sendyk
    @cite https://wiki.python.org/moin/UdpCommunication
    """

    UDP_IP = "10.147.20.134"
    UDP_PORT = 5515
    TOLERANCE = 5

    def __init__(self, controller_type: int):
        """
        Initializes the local objects

        :param controller_type: Integer representing the type of Controller used to transmit information
                                1 - Gamepad Controller [Default]
                                2 - HP Reverb G2
                                3 - Keyboard
        """
        self.controller_type = controller_type

        match controller_type:
            case 1:
                self.game_controller = GamepadController()
            case 2:
                self.game_controller = VRController()
            case 3:
                self.game_controller = KeyboardController()
            case _:
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
        l_r_val = int(unprocessed_message[0] * 255)
        f_b_val = int(unprocessed_message[1] * 255)

        if abs(l_r_val) > abs(f_b_val):
            if l_r_val > self.TOLERANCE:
                message += "R" + "{:03d}".format(abs(l_r_val))
            elif l_r_val < -self.TOLERANCE:
                message += "L" + "{:03d}".format(abs(l_r_val))
            else:
                message += "N" + "{:03d}".format(abs(l_r_val))
        else:
            if f_b_val > self.TOLERANCE:
                message += "F" + "{:03d}".format(abs(f_b_val))
            elif f_b_val < -self.TOLERANCE:
                message += "B" + "{:03d}".format(abs(f_b_val))
            else:
                message += "N" + "{:03d}".format(abs(f_b_val))

        msg = bytes(message, self.bytes_format)

        return msg


if __name__ == '__main__':
    ControlTransmission(1).transmit_udp()
