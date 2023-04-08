import json

import numpy as np
import cv2
import av
from stereovision.calibration import StereoCalibration
from enum import Enum


class State(Enum):
    """
    Enum representing the state of the obstacle avoidance state machine.

    NOTE:
    Please refer to algorithm documentation to understand state references.
    Documentation is available in the State_Machine.png file
    """
    A = 1
    B = 2
    C = 3
    D = 4
    E = 5
    F = 6
    G = 7
    H = 8
    I = 9
    J = 10
    DONE = 11


class ObstacleAvoidance:
    """
    Object which works to asynchronously update state variables and react as a state machine in a dynamic environment

    GOAL:
    Find, and track a stop sign.  Avoid obstacles on the way to meeting this end destination.

    @author Nicholas Sendyk, 101143602
    @cite https://www.researchgate.net/publication/333573350_Finite_state_automaton_based_control_system_for_walking_machines
    @cite https://stereopi.com/blog/opencv-and-depth-map-stereopi-tutorial
    """

    UDP_IP = "10.147.20.134"
    UDP_PORT = 5515

    def __init__(self):
        # Initialize state variables
        self.state = State.A
        self.counter = 0
        self.stop_sign = False
        self.obstacle = True
        self.obstacle_is_stop = False

        # Initialize computer vision settings
        # Depth map default preset
        self.SWS = 5
        self.PFS = 5
        self.PFC = 29
        self.MDS = -30
        self.NOD = 160
        self.TTH = 100
        self.UR = 10
        self.SR = 14
        self.SPWS = 100

        # Implementing calibration data
        print('Read calibration data and rectifying stereo pair...')
        self.calibration = StereoCalibration(input_folder='calib_result')


        # Initialize the video stream
        # create a PyAV input stream from the raw H264 video stream
        self.container = av.open("udp://192.168.0.218:3001?overrun_nonfatal=1&fifo_size=50000&pkt_size=30000")
        self.stream = self.container.streams.video[0]

        # Wait for the video stream to load


    def _monitor_video_stream(self) -> None:
        return

    def calculate_iou(self, bbox1, bbox2):
        """

        :param bbox1:
        :param bbox2:
        :return:
        """
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        if x1 >= x2 + w2 or x2 >= x1 + w1 or y1 >= y2 + h2 or y2 >= y1 + h1:
            return 0.0
        else:
            intersect_x1 = max(x1, x2)
            intersect_y1 = max(y1, y2)
            intersect_x2 = min(x1 + w1, x2 + w2)
            intersect_y2 = min(y1 + h1, y2 + h2)
            intersect_area = (intersect_x2 - intersect_x1) * (intersect_y2 - intersect_y1)
            bbox1_area = w1 * h1
            bbox2_area = w2 * h2
            iou = intersect_area / float(bbox1_area)  # + bbox2_area - intersect_area)
            return iou

    def a(self) -> bool:
        """

        :return:
        """
        self.counter = 0

        if self.stop_sign:
            self.state = State.E
        else:
            self.state = State.B

        return True

    def b(self) -> bool:
        """

        :return:
        """
        self.counter = self.counter + 1
        # TODO: TURN LEFT
        # WAIT FOR PERIOD OF TIME

        if self.stop_sign:
            self.state = State.E
        elif self.counter == 20:
            self.state = State.C
        else:
            self.state = State.B

        return True

    def c(self) -> bool:
        """

        :return:
        """
        if self.obstacle:
            self.state = State.B
        else:
            self.state = State.D

        return True

    def d(self) -> bool:
        """

        :return:
        """
        # TODO: DRIVE FORWARDS AND THEN STOP
        # WAIT FOR PERIOD OF TIME

        self.state = State.A

        return True

    def e(self) -> bool:
        """

        :return:
        """
        if self.obstacle and self.stop_sign:
            self.state = State.G
        elif self.obstacle and not self.stop_sign:
            self.state = State.H
        elif not self.obstacle and self.stop_sign:
            self.state = State.F
        else:
            self.state = State.A

        return True

    def f(self) -> bool:
        """

        :return:
        """
        # TODO: DRIVE FORWARDS AND THEN STOP
        # WAIT FOR PERIOD OF TIME

        self.state = State.E

        return True

    def g(self) -> bool:
        """

        :return:
        """
        if self.obstacle_is_stop:
            self.state = State.DONE
        else:
            self.state = State.H

        return True

    def h(self) -> bool:
        """

        :return:
        """
        if self.obstacle_is_stop:
            self.state = State.DONE
        elif self.obstacle:
            self.state = State.I
        else:
            self.state = State.E

        return True

    def i(self) -> bool:
        """

        :return:
        """
        # TODO: TURN LEFT AND THEN STOP
        # WAIT FOR PERIOD OF TIME
        if self.obstacle:
            self.state = State.I
        else:
            self.state = State.J

        return True

    def j(self) -> bool:
        """

        :return:
        """
        # TODO: DRIVE FORWARD AND LOOK RIGHT
        # WAIT FOR PERIOD OF TIME

        self.state = State.H

        return True

    def run(self):
        """
        Runs the automated state machine

        :return:
        """
        while self.state is not State.DONE:
            match self.state:
                case State.A:
                    self.a()
                case State.B:
                    self.b()
                case State.C:
                    self.c()
                case State.D:
                    self.d()
                case State.E:
                    self.e()
                case State.F:
                    self.f()
                case State.G:
                    self.g()
                case State.H:
                    self.h()
                case State.I:
                    self.i()
                case State.J:
                    self.j()
                case _:
                    continue

        # TODO: Close the opencv connection
        cv2.destroyAllWindows()
