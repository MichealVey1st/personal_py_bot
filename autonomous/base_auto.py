from magicbot.state_machine import state, timed_state, AutonomousStateMachine
from components import swervedrive

class BaseAuto(AutonomousStateMachine):
    drive: swervedrive.SwerveDrive

    @state
    def failed(self):
        # should be called when it FAILS
        self.drive.debug(debug_modules=True)
        self.next_state('finish')

    @state
    def finish(self):
        self.drive.flush()
        self.done()