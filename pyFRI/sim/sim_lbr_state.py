import pybullet as p


class SimLBRState:
    def __init__(self):
        self._sample_time = None
        self._session_state = None
        self._measured_joint_position = None
        self._measured_torque = None
        self._external_torque = None
        self._ipo_joint_position = None
        self._client_command_mode = None

    # Set methods

    def set_sample_time(self, sample_time):
        self._sample_time = sample_time

    def set_session_state(self, session_state):
        self._session_state = session_state

    def set_measured_joint_position(self, position):
        self._measured_joint_position = position

    def set_measured_torque(self, torque):
        self._measured_torque = torque

    def set_external_torque(self, ext_torque):
        self._external_torque = ext_torque

    def set_ipo_joint_position(self, position):
        self._ipo_joint_position = position

    # Get methods

    def getSampleTime(self):
        return self._sample_time

    def getSessionState(self):
        return self._session_state

    def getClientCommandMode(self):
        return self._client_command_mode

    def getMeasuredJointPosition(self):
        return self._measured_joint_position

    def getMeasuredTorque(self):
        return self._measured_torque

    def getExternalTorque(self):
        return self._external_torque

    def getIpoJointPosition(self):
        return self._ipo_joint_position
