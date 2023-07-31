class SimLBRCommand:
    def __init__(self):
        self.reset()

    def reset(self):
        self._position = None
        self._wrench = None
        self._torque = None

    def setJointPosition(self, values):
        self._position = values

    def setWrench(self, wrench):
        self._wrench = wrench

    def setTorque(self, torques):
        self._torques = torques

    def get_joint_position(self):
        assert self._position is not None, "Position command not set!"
        return self._position

    def get_wrench(self):
        assert self._wrench is not None, "Wrench command not set!"
        return self._wrench

    def get_torques(self):
        assert self._torques is not None, "Torque command not set!"
        return self._torques
