from .base_auto import BaseAuto
from magicbot.state_machine import state, timed_state, AutonomousStateMachine
from components import swervedrive

class Default(BaseAuto):
    MODE_NAME = "Default"
    DEFAULT = True

    drive: swervedrive.SwerveDrive

    @timed_state(duration=1.5, first=True, next_state="back")
    def forward(self):
        self.drive.set_raw_fwd(-0.5)
        self.drive.set_raw_rcw(0.5)
    
    # Drive backwards.
    @timed_state(duration=1.5, next_state="finish")
    def back(self):
        self.drive.set_raw_fwd(0.5)

class Pathed(BaseAuto):
    MODE_NAME = "Pathed"
    DEFAULT = False

    drive: swervedrive.SwerveDrive

    # TODO
    # NEED to add specific distances 