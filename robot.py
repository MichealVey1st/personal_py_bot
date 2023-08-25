import wpilib
import ctre
from magicbot import MagicRobot
from robotpy_ext.autonomous.selector import AutonomousModeSelector
from networktables import NetworkTables
from networktables.util import ntproperty
from components import swervedrive, swervemodule


from collections import namedtuple
# Get the config preset from the swervemodule
ModuleConfig = swervemodule.ModuleConfig

class MyRobot(MagicRobot):

    drive: swervedrive.SwerveDrive

    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule

if __name__ == "__main__":
    wpilib.run(MyRobot)
