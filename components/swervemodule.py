import math
import wpilib
import ctre

from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

MAX_VOLTAGE = 5

class SwerveModule:
    driveMotor: ctre.WPI_TalonFX
    rotateMotor: ctre.WPI_TalonFX

    encoder: wpilib.AnalogInput

    cfg: ModuleConfig

    def setup(self):
        self.sd_prefix = self.cfg.sd_prefix or 'Module'
        self.encoder_zero = self.cfg.zero or 0
        self.inverted = self.cfg.inverted or False
        self.allow_reverse = self.cfg.allow_reverse or True

        # Smart dashboard to see what is happening

        self.sd = NetworkTables.getTable('SmartDashboard')
        self.debugging = self.sd.getEntry('drive/drive/debugging')

        self.driveMotor.setInverted(self.inverted)

        self._requested_voltage = 0
        self._requested_speed = 0

        # PID control
        # kP = 1.5, kI = 0.0, kD = 0.0
        self._pid_controller = PIDController(1.5, 0.0, 0.0)
        self._pid_controller.enableContinuousInput(0.0, 5.0) #sets 0 and 5 as the same point?
        self._pid_controller.setTolerance(0.05, 0.05)# tolerence to Align PID?

    def get_voltage(self):
        return self.encoder.getAverageVoltage() - self.encoder_zero
    
    def flush(self):
        self._requested_voltage = self.encoder_zero
        self._requested_speed = 0
        self._pid_controller.reset()

    @staticmethod
    def voltage_to_degrees(voltage):
        deg = (voltage / 5) * 360
        if deg < 0:
            deg += 360
        return deg
    
    @staticmethod
    def voltage_to_rad(voltage):
        return (voltage / 5) * 2 * math.pi

    @staticmethod
    def degree_to_voltage(degree):
        return (degree / 360) * 5
    
    def _set_deg(self, value):
        self._requested_voltage = ((self.degree_to_voltage(value) + self.encoder_zero) % 5)

    def move(self, speed, deg):
        deg %= 360

        if self.allow_reverse:
            # If difference is more than 90 deg dont turn the wheel 180 reverse the speed instead
            if abs(deg - self.voltage_to_degrees(self.get_voltage())):
                speed *= -1
                deg =+ 180
                deg %= 360
        self._requested_speed = speed
        self._set_deg(deg)

    def debug(self):
        print(self.sd_prefix, '; requested_speed: ', self._requested_speed, ' requested_voltage: ', self._requested_voltage)

    def execute(self):
        error = self._pid_controller.calculate(self.encoder.getVoltage(), self._requested_voltage)

        output = 0

        if not self._pid_controller.atSetpoint():
            output = max(min(error, 1), -1)

        # Put the output to the dashboard
        self.sd.putNumber('drive/%s/output' % self.sd_prefix, output)
        # Set the output as the rotateMotor's voltage
        self.rotateMotor.set(output)

        #set requested speed as the drive motor voltage
        self.driveMotor.set(self._requested_speed)

        self.update_smartdash()
    
    def update_smartdash(self):
        self.sd.putNumber('drive/%s/degrees' % self.sd_prefix, self.voltage_to_degrees(self.get_voltage()))

        if self.debugging.getBoolean(False):
            self.sd.putNumber('drive/%s/requested_voltage' % self.sd_prefix, self._requested_voltage)
            self.sd.putNumber('drive/%s/requested_speed' % self.sd_prefix, self._requested_speed)
            self.sd.putNumber('drive/%s/raw voltage' % self.sd_prefix, self.encoder.getVoltage())  # DO NOT USE self.get_voltage() here
            self.sd.putNumber('drive/%s/average voltage' % self.sd_prefix, self.encoder.getAverageVoltage())
            self.sd.putNumber('drive/%s/encoder_zero' % self.sd_prefix, self.encoder_zero)

            self.sd.putNumber('drive/%s/PID Setpoint' % self.sd_prefix, self._pid_controller.getSetpoint())
            self.sd.putNumber('drive/%s/PID Error' % self.sd_prefix, self._pid_controller.getPositionError())
            self.sd.putBoolean('drive/%s/PID isAligned' % self.sd_prefix, self._pid_controller.atSetpoint())

            self.sd.putBoolean('drive/%s/allow_reverse' % self.sd_prefix, self.allow_reverse)
