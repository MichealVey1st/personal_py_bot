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

    frontLeftModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=2.97, inverted=True, allow_reverse=True)
    frontRightModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=2.69, inverted=False, allow_reverse=True)
    rearLeftModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0.18, inverted=True, allow_reverse=True)
    rearRightModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=4.76, inverted=False, allow_reverse=True)

    def createObjects(self):
        self.sd = NetworkTables.getTable('SmartDashboard')

        self.gamempad = wpilib.Joystick(0)
        self.gamempad2 = wpilib.Joystick(1)

        # WPI_TalonFX

        # Drive Motors
        self.frontLeftModule_driveMotor = ctre.WPI_TalonFX(5)
        self.frontRightModule_driveMotor = ctre.WPI_TalonFX(8)
        self.rearLeftModule_driveMotor = ctre.WPI_TalonFX(4)
        self.rearRightModule_driveMotor = ctre.WPI_TalonFX(9)

        # Rotate Motors
        self.frontLeftModule_rotateMotor = ctre.WPI_TalonFX(3)
        self.frontRightModule_rotateMotor = ctre.WPI_TalonFX(14)
        self.rearLeftModule_rotateMotor = ctre.WPI_TalonFX(2)
        self.rearRightModule_rotateMotor = ctre.WPI_TalonFX(15)

        # Encoders
        self.frontLeftModule_encoder = wpilib.AnalogInput(0)
        self.frontRightModule_encoder = wpilib.AnalogInput(3)
        self.rearLeftModule_encoder = wpilib.AnalogInput(1)
        self.rearRightModule_encoder = wpilib.AnalogInput(2)

        # Limit Switch
        self.switch = wpilib.DigitalInput(0)

        # PDP
        self.pdp = wpilib.PowerDistributionPanel(0)

    def disabledPeriodic(self):
        self.update_sd()

    def autonomousInit(self):
        self.drive.flush()
        self.drive.threshold_input_vectors = True

    def autonomous(self):
        super().autonomous()

    def teleopInit(self):
        self.drive.flush()
        self.drive.squared_inputs = True
        self.drive.threshold_input_vectors = True
    
    def move(self, x, y, rcw):
        # velocity in each axis XYZ

        if self.gamempad.getRawButton(3):
            rcw *= 0.7

        self.drive.move(x, y, rcw)

    def teleopPeriodic(self):
        # drive
        self.move(self.gamempad.getRawAxis(5), self.gamempad.getRawAxis(4), self.gamempad.getRawAxis(0))

        #lock
        if self.gamempad.getRawButton(1):
            self.drive.request_wheel_lock = True

        #button drive
        if self.gamempad.getPOV() == 0:
            self.drive.set_raw_fwd(-0.35)
        elif self.gamempad.getPOV() == 180:
            self.drive.set_raw_fwd(0.35)
        elif self.gamempad.getPOV() == 90:
            self.drive.set_raw_strafe(0.35)
        elif self.gamempad.getPOV() == 270:
            self.drive.set_raw_strafe(-0.35)
    
        self.update_sd()

    def update_sd(self):
        self.drive.update_smartdash()
        


if __name__ == "__main__":
    wpilib.run(MyRobot)
