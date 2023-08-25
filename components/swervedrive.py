import math
from magicbot import magiccomponent
from components import swervemodule
from networktables import NetworkTable
from networktables.util import ntproperty

class SwerveDrive:
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule:swervemodule.SwerveModule
    rearLeftModule:swervemodule.SwerveModule
    rearRightModule:swervemodule.SwerveModule


    # Get some config options from the dashboard.
    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.1)
    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 0.5)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 0.65)
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', False) # Turn to true to run it in verbose mode.

    def setup(self):
        self.modules = {
            'front_left': self.frontLeftModule,
            'front_right': self.frontRightModule,
            'rear_left': self.rearLeftModule,
            'rear_right': self.rearRightModule
        }
        
        self.sd = NetworkTable.getTable('SmartDashboard')

        self._requested_vectors =