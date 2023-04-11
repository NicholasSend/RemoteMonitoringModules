import threading
import time
from timeit import default_timer

import numpy as np
import openvr


class ReverbG2(object):
    """
    A multithreaded object for Monitoring Reverb G2 Head Tracking and allowing real time input access

    @ author Nicholas Sendyk, 101143602
    @cite https://readthedocs.org/projects/inputs/downloads/pdf/latest/
    @cite https://github.com/kevinhughes27/TensorKart/
    """

    def __init__(self):
        """
        Initialization of Gamepad Controller Inputs followed by initialization of threads.
        """

        openvr.init(openvr.VRApplication_Scene)
        self.base = None
        self.poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
        self.poses = self.poses_t()
        self.start = default_timer()
        self.is_first_run = True
        self.lookLeft = 0
        self.lookRight = 0
        self.lookUp = 0
        self.lookDown = 0
        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        """
        Accesses current Head Tracking state for a subset of directionsg.

        :return: List[Float, Float, Float, Float]
        """
        l = self.lookLeft
        r = self.lookRight
        u = self.lookUp
        d = self.lookDown

        return [l, r, u, d]

    def _monitor_controller(self):
        """
        A controller button event monitoring thread

        :return: None
        """
        i = 0
        while True:
            openvr.VRCompositor().waitGetPoses(self.poses, None)
            i = i + 1
            hmd_pose = self.poses[openvr.k_unTrackedDeviceIndex_Hmd]
            if hmd_pose.bPoseIsValid:
                hmd_orientation = hmd_pose.mDeviceToAbsoluteTracking

                if self.is_first_run:
                    self.is_first_run = not self.is_first_run
                    self.base = np.array([
                        np.array(
                            [float(hmd_orientation[0][0]), float(hmd_orientation[0][1]), float(hmd_orientation[0][2])]),
                        np.array(
                            [float(hmd_orientation[1][0]), float(hmd_orientation[1][1]), float(hmd_orientation[1][2])]),
                        np.array(
                            [float(hmd_orientation[2][0]), float(hmd_orientation[2][1]), float(hmd_orientation[2][2])])
                    ])

                if ((self.base[2][1] - hmd_orientation[2][1]) <= -0.5) and (
                        self.base[1][2] - hmd_orientation[1][2] >= 0.5):
                    self.lookUp = 1
                else:
                    self.lookUp = 0

                if (self.base[2][1] - hmd_orientation[2][1] >= 0.5) and (
                        self.base[1][2] - hmd_orientation[1][2] <= -0.5):
                    self.lookDown = 1
                else:
                    self.lookDown = 0

                if (self.base[2][0] - hmd_orientation[2][0] >= 0.5) and (
                        self.base[0][2] - hmd_orientation[0][2] <= -0.5):
                    self.lookLeft = 1
                else:
                    self.lookLeft = 0

                if (self.base[2][0] - hmd_orientation[2][0] <= -0.5) and (
                        self.base[0][2] - hmd_orientation[0][2] >= 0.5):
                    self.lookRight = 1
                else:
                    self.lookRight = 0

            # for i in range(openvr.k_unMaxTrackedDeviceCount):
            #     device_class = openvr.VRSystem().getTrackedDeviceClass(i)
            #
            #     if device_class == openvr.TrackedDeviceClass_Controller:
            #         device_state = openvr.VRSystem().getControllerState(i)[1]
            #         if device_state.ulButtonPressed != 0:
            #             print("Controller button pressed: ", device_state.ulButtonPressed)
            #         if device_state.ulButtonTouched != 0:
            #             print("Controller button touched: ", device_state.ulButtonTouched)
            #         if device_state.rAxis[0].x != 0 or device_state.rAxis[0].y != 0:
            #             print("Controller joystick 0: ", device_state.rAxis[0].x, device_state.rAxis[0].y)
            #         if device_state.rAxis[1].x != 0 or device_state.rAxis[1].y != 0:
            #             print("Controller joystick 1: ", device_state.rAxis[1].x, device_state.rAxis[1].y)

            time.sleep(0.1)

    def shutdown(self):
        openvr.shutdown()
