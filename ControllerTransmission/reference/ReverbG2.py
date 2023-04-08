import math
import threading

import openvr


class VRController(object):
    """
    A multithreaded object for Monitoring VR Controller Output and allowing real time input access

    @author: Nicholas Sendyk
    @cite https://github.com/ValveSoftware/openvr/blob/master/samples/python/hellovr_py/hellovr_py.py
    """

    def __init__(self):
        self.hmd = None
        self.left_controller = None
        self.right_controller = None

        self.left_button_state = {}
        self.right_button_state = {}

        self.headset_pose = {}
        self.left_controller_pose = {}
        self.right_controller_pose = {}

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        """
        Accesses current VR state for each controller button and headset orientation.

        format:
            {
                "left_button_state": {
                    "system": False,
                    "application_menu": False,
                    "grip": False,
                    "menu": False,
                    "touchpad": False,
                    "trigger": False,
                },
                "right_button_state": {
                    "system": False,
                    "application_menu": False,
                    "grip": False,
                    "menu": False,
                    "touchpad": False,
                    "trigger": False,
                },
                "headset_pose": {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "pitch": 0,
                    "yaw": 0,
                    "roll": 0,
                },
                "left_controller_pose": {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "pitch": 0,
                    "yaw": 0,
                    "roll": 0,
                },
                "right_controller_pose": {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "pitch": 0,
                    "yaw": 0,
                    "roll": 0,
                }
            }

        :return: dictionary containing VR controller and headset information
        """
        return {
            "left_button_state": self.left_button_state,
            "right_button_state": self.right_button_state,
            "headset_pose": self.headset_pose,
            "left_controller_pose": self.left_controller_pose,
            "right_controller_pose": self.right_controller_pose,
        }

    def _monitor_controller(self):
        """
        A controller button event monitoring thread

        :return: None
        """
        openvr.init(openvr.VRApplication_Other)
        while True:
            # Handle VR events
            vr_events = openvr.VREvent_t()
            while openvr.VRSystem().PollNextEvent(vr_events):
                pass

            # Get left and right controller state and pose
            left_controller_id = openvr.k_unTrackedDeviceIndexInvalid
            right_controller_id = openvr.k_unTrackedDeviceIndexInvalid
            pose = openvr.TrackedDevicePose_t()
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                device_class = openvr.VRSystem().GetTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_Controller:
                    if openvr.VRSystem().GetControllerRoleForTrackedDeviceIndex(
                            i) == openvr.TrackedControllerRole_LeftHand:
                        left_controller_id = i
                    elif openvr.VRSystem().GetControllerRoleForTrackedDeviceIndex(
                            i) == openvr.TrackedControllerRole_RightHand:
                        right_controller_id = i
                elif device_class == openvr.TrackedDeviceClass_HMD:
                    pose = openvr.VRSystem().GetDeviceToAbsoluteTrackingPose(device_class, 0, pose)

            # Handle left controller events
            if left_controller_id != openvr.k_unTrackedDeviceIndexInvalid:
                controller_state = openvr.VRSystem().GetControllerState(left_controller_id)
                self.left_button_state["system"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_System)) != 0
                self.left_button_state["application_menu"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_ApplicationMenu)) != 0
                self.left_button_state["grip"] = (controller_state.ulButtonPressed & (1 << openvr.k_EButton_Grip)) != 0
                self.left_button_state["menu"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_SteamVR_Trigger)) != 0
                self.left_button_state["touchpad"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_SteamVR_Touchpad)) != 0
                self.left_button_state["trigger"] = controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_SteamVR_Trigger)

                controller_pose = openvr.VRSystem().GetControllerStateWithPose(openvr.TrackingUniverseStanding,
                                                                               left_controller_id, pose)
                self.left_controller_pose["x"] = controller_pose.mDeviceToAbsoluteTracking[0][3]
                self.left_controller_pose["y"] = controller_pose.mDeviceToAbsoluteTracking[1][3]
                self.left_controller_pose["z"] = controller_pose.mDeviceToAbsoluteTracking[2][3]
                _, pitch, yaw = openvr.utils.rotationMatrixToEulerAngles(controller_pose.mDeviceToAbsoluteTracking)
                self.left_controller_pose["pitch"] = pitch
                self.left_controller_pose["yaw"] = yaw
                self.left_controller_pose["roll"] = 0

            # Handle right controller events
            if right_controller_id != openvr.k_unTrackedDeviceIndexInvalid:
                controller_state = openvr.VRSystem().GetControllerState(right_controller_id)
                self.right_button_state["system"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_System)) != 0
                self.right_button_state["application_menu"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_ApplicationMenu)) != 0
                self.right_button_state["grip"] = (controller_state.ulButtonPressed & (1 << openvr.k_EButton_Grip)) != 0
                self.right_button_state["menu"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_SteamVR_Touchpad)) != 0
                self.right_button_state["touchpad"] = (controller_state.ulButtonPressed & (
                        1 << openvr.k_EButton_SteamVR_Touchpad)) != 0
                self.right_button_state["trigger"] = controller_state.rAxis[1].x > 0.5

                # Get right controller pose
                pose = openvr.TrackedDevicePose_t()
                pose = openvr.VRSystem().GetDeviceToAbsoluteTrackingPose(right_controller_id,
                                                                         openvr.k_unMaxTrackedDevicePoseAgeSeconds)
                if pose.bPoseIsValid:
                    self.right_controller_pose["x"] = pose.mDeviceToAbsoluteTracking[0][3]
                    self.right_controller_pose["y"] = pose.mDeviceToAbsoluteTracking[1][3]
                    self.right_controller_pose["z"] = pose.mDeviceToAbsoluteTracking[2][3]
                    _, self.right_controller_pose["yaw"], _ = self.mat2euler(pose.mDeviceToAbsoluteTracking)
                    self.right_controller_pose["pitch"] = pose.mDeviceToAbsoluteTracking[1][0]
                    self.right_controller_pose["roll"] = pose.mDeviceToAbsoluteTracking[2][0]

            # Get headset pose
            pose = openvr.TrackedDevicePose_t()
            pose = openvr.VRSystem().GetDeviceToAbsoluteTrackingPose(openvr.k_unTrackedDeviceIndex_Hmd,
                                                                     openvr.k_unMaxTrackedDevicePoseAgeSeconds)
            if pose.bPoseIsValid:
                self.headset_pose["x"] = pose.mDeviceToAbsoluteTracking[0][3]
                self.headset_pose["y"] = pose.mDeviceToAbsoluteTracking[1][3]
                self.headset_pose["z"] = pose.mDeviceToAbsoluteTracking[2][3]
                _, self.headset_pose["yaw"], _ = self.mat2euler(pose.mDeviceToAbsoluteTracking)
                self.headset_pose["pitch"] = pose.mDeviceToAbsoluteTracking[1][0]
                self.headset_pose["roll"] = pose.mDeviceToAbsoluteTracking[2][0]

    def mat2euler(self, mat):
        """
        Utility function to convert a 3x4 transformation matrix to Euler angles

        :param mat: 3x4 transformation matrix
        :return: tuple of Euler angles
        """
        pitch = -math.asin(mat[1][2])
        yaw = math.atan2(mat[0][2], mat[2][2])
        roll = math.atan2(mat[1][0], mat[1][1])

        return pitch, yaw, roll


if __name__ == "__main__":
    controller = VRController()
    while True:
        vr_state = controller.read()
        print(vr_state)

    openvr.shutdown()
