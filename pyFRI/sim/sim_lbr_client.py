import abc
from .sim_lbr_state import SimLBRState
from .sim_lbr_command import SimLBRCommand


class SimLBRClient(abc.ABC):
    def __init__(self):
        self._lbr_state = SimLBRState()
        self._lbr_command = SimLBRCommand()

    def robotState(self):
        return self._lbr_state

    def robotCommand(self):
        return self._lbr_command

    @abc.abstractmethod
    def monitor(self):
        pass

    @abc.abstractmethod
    def onStateChange(self, old_state, new_state):
        pass

    @abc.abstractmethod
    def waitForCommand(self):
        pass

    @abc.abstractmethod
    def command(self):
        pass
