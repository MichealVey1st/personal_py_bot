import wpilib
from wpilib.drive import RobotDriveBase
from robotpy_ext.autonomous import StatefulAutonomous, state, timed_state
from autonomous.drive_forward import DriveForward

# Import classes from your swerve.py file
from swerve import Translation2d, Rotation2d, ChassisSpeeds, SwerveModuleState, SwerveDriveKinematics

class MyRobotDrive(RobotDriveBase):
    # Define your drive methods here
    pass

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        # Rest of your robot initialization code
        
        # Create module location objects
        m_frontLeftLocation = Translation2d(0.381, 0.381)
        m_frontRightLocation = Translation2d(0.381, -0.381)
        m_backLeftLocation = Translation2d(-0.381, 0.381)
        m_backRightLocation = Translation2d(-0.381, -0.381)

        # Creating kinematics object using module locations
        self.m_kinematics = SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        )
        
        # Example chassis speeds
        self.speeds = ChassisSpeeds(1.0, 3.0, 1.5)

        # Create a RobotDriveBase instance
        self.drive = MyRobotDrive()  # Replace with your actual RobotDriveBase subclass
        
        # Create an autonomous instance
        self.autonomous = DriveForward(self.drive, self.speeds)  # Replace with actual parameters


    def teleopPeriodic(self):
        # Convert chassis speeds to module states
        module_states = self.m_kinematics.toSwerveModuleStates(self.speeds)
        
        # Example optimization
        current_angle = Rotation2d(0.0)
        front_left_optimized = SwerveModuleState.optimize(module_states[0], current_angle)
        
        # Convert module states to chassis speeds
        chassis_speeds = self.m_kinematics.toChassisSpeeds(*module_states)
        
        # Get individual speeds from chassis speeds
        forward = chassis_speeds.vxMetersPerSecond
        sideways = chassis_speeds.vyMetersPerSecond
        angular = chassis_speeds.omegaRadiansPerSecond
        
        # Apply the speeds to your RobotDriveBase instance (replace with your actual motor controllers)
        # For example:
        self.drive.arcadeDrive(forward, angular)
    
    def autonomousInit(self):
        self.autonomous.start()

    def autonomousPeriodic(self):
        self.autonomous.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
