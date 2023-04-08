import openvr
from scipy.spatial.transform import Rotation


class VRController(object):
    """
    An object for monitoring VR controller and headset inputs.

    @author Nicholas Sendyk, 101143602
    """

    def __init__(self):
        """
        Initialization of VR controller and headset inputs.
        """
        self.controllers = {}
        self.hmd = None
        self.vr_system = None

        self._initialize()

    def _initialize(self):
        """
        Initializes VR controller and headset inputs.
        """
        openvr.init(openvr.VRApplication_Other)

        # Get the VR system
        self.vr_system = openvr.VRSystem()

        # Get the number of tracked devices
        num_devices = self.vr_system.getTrackedDeviceCount()

        # Loop through all tracked devices
        for i in range(num_devices):
            device_class = self.vr_system.getTrackedDeviceClass(i)

            if device_class == openvr.TrackedDeviceClass.Controller:
                # Add a new controller
                self.controllers[i] = {
                    'trigger': 0,
                    'trackpad_x': 0,
                    'trackpad_y': 0,
                    'grip': 0,
                    'menu': 0,
                }
            elif device_class == openvr.TrackedDeviceClass.HMD:
                # Set the HMD
                self.hmd = i

    def read(self):
        """
        Accesses current VR controller and headset state.

        :return: dictionary with controller and headset state
        """
        vr_events = self.vr_system.getEvent()
        for vr_event in vr_events:
            event_type = vr_event.eventType
            device_type = self.vr_system.getTrackedDeviceClass(vr_event.trackedDeviceIndex)

            # Update controller state
            if device_type == openvr.TrackedDeviceClass.Controller:
                if event_type == openvr.VREvent_ButtonPress:
                    self.controllers[vr_event.trackedDeviceIndex][self._button_map(vr_event.data.controller.button)] = 1
                elif event_type == openvr.VREvent_ButtonUnpress:
                    self.controllers[vr_event.trackedDeviceIndex][self._button_map(vr_event.data.controller.button)] = 0
                elif event_type == openvr.VREvent_Axis:
                    if vr_event.data.controller.axis == 0:
                        self.controllers[vr_event.trackedDeviceIndex]['trackpad_x'] = vr_event.data.controller.x
                    elif vr_event.data.controller.axis == 1:
                        self.controllers[vr_event.trackedDeviceIndex]['trackpad_y'] = vr_event.data.controller.y
                    elif vr_event.data.controller.axis == 2:
                        self.controllers[vr_event.trackedDeviceIndex]['trigger'] = vr_event.data.controller.x
                    elif vr_event.data.controller.axis == 3:
                        self.controllers[vr_event.trackedDeviceIndex]['grip'] = vr_event.data.controller.x
                elif event_type == openvr.VREvent_ButtonTouch:
                    pass
                elif event_type == openvr.VREvent_ButtonUntouch:
                    pass

            # Update HMD state
            elif device_type == openvr.TrackedDeviceClass.HMD:
                if event_type == openvr.VREvent_TrackedDeviceActivated:
                    pass
                elif event_type == openvr.VREvent_TrackedDeviceDeactivated:
                    pass
                elif event_type == openvr.VREvent_TrackedDeviceUpdated:
                    pass

        # Get controller and headset state as dictionary and return it
        controller_state = self.controllers
        headset_state = self.hmd
        return {'controller_state': controller_state, 'headset_state': headset_state}


# Initialize OpenVR system
openvr.init(openvr.VRApplication_Background)

# Create VR system and compositor objects
vr_system = openvr.VRSystem()
vr_compositor = openvr.VRCompositor()

# Create VR controller objects
left_controller = VRController(vr_system, openvr.TrackedControllerRole_LeftHand)
right_controller = VRController(vr_system, openvr.TrackedControllerRole_RightHand)

# Continuously poll VR system for events and update controller and headset state
while True:
    # Poll VR system for events
    openvr.VRSystem().pollNextEvent()
    # Update controller and headset state
    left_controller.read()
    right_controller.read()
    hmd_state = vr_system.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                          openvr.k_unMaxTrackedDeviceCount)

    # Update headset state
    if hmd_state[openvr.k_unTrackedDeviceIndex_Hmd].bPoseIsValid:
        hmd_pose = hmd_state[openvr.k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking
        hmd_position = hmd_pose[:3, 3]
        hmd_rotation = Rotation.from_matrix(hmd_pose[:3, :3]).as_quat()
        headset_state = {'position': hmd_position, 'rotation': hmd_rotation}

    # Submit VR frames to compositor
    vr_compositor.submit(openvr.Eye_Left, left_controller.texture_handle)
    vr_compositor.submit(openvr.Eye_Right, right_controller.texture_handle)
