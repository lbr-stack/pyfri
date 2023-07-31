import pybullet as p
import pyFRI as fri
from copy import deepcopy

if fri.FRI_VERSION_MAJOR == 1:
    CLIENT_COMMAND_MODE_POSITION = fri.EClientCommandMode.POSITION
elif fri.FRI_VERSION_MAJOR == 2:
    CLIENT_COMMAND_MODE_POSITION = fri.EClientCommandMode.JOINT_POSITION


class SimClientApplication:
    def __init__(self, client, lbr_config):
        self._client = client
        self._lbr_config = lbr_config
        self._state = fri.ESessionState.IDLE
        self._lbr_id = 0  # robot is always created first, thus will have ID 0
        self._lbr_actuated_joint_ids = []

    def _get_state(self):
        return deepcopy(self._state)

    def _step_pybullet(self):
        pass

    def _command_robot(self):
        # Command robot
        ccm = self._client.robotState().getClientCommandMode()
        if ccm == CLIENT_COMMAND_MODE_POSITION:
            p.setJointMotorControlArray(
                self._lbr_id,
                self._lbr_actuated_joint_ids,
                controlMode=p.POSITION_CONTROL,
                targetPositions=self._client.robotCommand().get_position().tolist(),
            )
        elif ccm == fri.EClientCommandMode.WRENCH:
            print("Warn: WRENCH not implemented")
        elif ccm == fri.EClientCommandMode.TORQUE:
            print("Warn: TORQUE not implemented")

        # Reset command buffer
        self._client.robotCommand().reset()

    def connect(self, port, hostname):
        self._client_id = p.connect(p.SHARED_MEMORY)
        if self._client_id == -1:
            print("Error: failed to connect to simulator.")
            return False

        # Get joint indices for actuated joints
        for joint_index in p.getNumJoints(self._lbr_id):
            info = p.getJointInfo(self._lbr_id, joint_index)
            if info[2] == p.JOINT_REVOLUTE:
                self._lbr_actuated_joint_ids.append(joint_index)

        # Get physics parameters
        self._physics_params = p.getPhysicsEngineParameters()
        self._client.robotState().set_sample_time(self._physics_params['fixedSampleTime'])

        return True

    def step(self):
        # Check connection
        if not p.isConnected(self._client_id):
            print("Error: client application is not connected!")
            return False

        # Handle robot state
        current_state = self._get_state()
        if current_state == fri.ESessionState.IDLE:
            old_state = current_state
            new_state = fri.ESessionState.MONITORING_WAIT
            self._state = new_state
            self._client.onStateChange(old_state, new_state)
            self._step_pybullet()
            return True
        elif current_state == fri.ESessionState.MONITORING_WAIT:
            old_state = current_state
            new_state = fri.ESessionState.MONITORING_READY
            self._state = new_state
            self._client.onStateChange(old_state, new_state)
            self._step_pybullet()
            return True
        elif current_state == fri.ESessionState.MONITORING_READY:
            old_state = current_state
            new_state = fri.ESessionState.COMMANDING_WAIT
            self._client.onStateChange(old_state, new_state)
            self._client.monitor()
            return True
        elif current_state == fri.ESessionState.COMMANDING_WAIT:
            old_state = current_state
            new_state = fri.ESessionState.COMMANDING_ACTIVE
            self._client.onStateChange(old_state, new_state)
            self._client.waitForCommand()
        elif current_state == fri.ESessionState.COMMANDING_ACTIVE:
            self._client.command()

        # Send command to robot
        self._command_robot()

    def disconnect(self):
        p.disconnect(self._client_id)
