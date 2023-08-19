import math

# Translation2d class representing a 2D translation
class Translation2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Rotation2d class representing a 2D rotation
class Rotation2d:
    def __init__(self, radians):
        self.radians = radians

# ChassisSpeeds class representing chassis speeds in various directions
class ChassisSpeeds:
    def __init__(self, vx, vy, omega):
        self.vxMetersPerSecond = vx
        self.vyMetersPerSecond = vy
        self.omegaRadiansPerSecond = omega

# SwerveModuleState class representing the state of a swerve module
class SwerveModuleState:
    def __init__(self, speed, angle):
        self.speedMetersPerSecond = speed
        self.angle = angle

    # Method to optimize module state angle and speed
    @staticmethod
    def optimize(state, current_angle):
        angle_diff = state.angle.radians - current_angle.radians

        # Ensure angle difference is within -pi to pi range
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Change speed to negative if angle difference is large
        new_speed = -state.speedMetersPerSecond if abs(angle_diff) > (math.pi / 2) else state.speedMetersPerSecond

        # Calculate new angle for optimized state
        new_angle = Rotation2d(current_angle.radians - angle_diff)

        return SwerveModuleState(new_speed, new_angle)

# SwerveDriveKinematics class for swerve drive kinematics calculations
class SwerveDriveKinematics:
    def __init__(self, *module_locations):
        self.module_locations = module_locations

    # Convert chassis speeds to swerve module states
    def toSwerveModuleStates(self, speeds):
        module_states = []
        for i in range(len(self.module_locations)):
            # Calculate module speed and angle based on chassis speeds and module locations
            module_speed = speeds.vxMetersPerSecond * math.cos(self.module_locations[i].y) + \
                           speeds.vyMetersPerSecond * math.sin(self.module_locations[i].x)
            module_angle = self.module_locations[i].y * speeds.vxMetersPerSecond - \
                           self.module_locations[i].x * speeds.vyMetersPerSecond

            module_states.append(SwerveModuleState(module_speed, Rotation2d(module_angle)))

        return module_states

# Create module location objects
m_frontLeftLocation = Translation2d(0.381, 0.381)
m_frontRightLocation = Translation2d(0.381, -0.381)
m_backLeftLocation = Translation2d(-0.381, 0.381)
m_backRightLocation = Translation2d(-0.381, -0.381)

# Creating kinematics object using module locations
m_kinematics = SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
)

# Example chassis speeds
speeds = ChassisSpeeds(1.0, 3.0, 1.5)

# Convert to module states
module_states = m_kinematics.toSwerveModuleStates(speeds)

# Example optimization
current_angle = Rotation2d(0.0)
front_left_optimized = SwerveModuleState.optimize(module_states[0], current_angle)

# Example field-oriented drive
speeds_field_oriented = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, math.pi / 2.0, Rotation2d.fromDegrees(45.0))
module_states_field_oriented = m_kinematics.toSwerveModuleStates(speeds_field_oriented)

# Convert module states to chassis speeds
chassis_speeds = m_kinematics.toChassisSpeeds(*module_states)

# Getting individual speeds
forward = chassis_speeds.vxMetersPerSecond
sideways = chassis_speeds.vyMetersPerSecond
angular = chassis_speeds.omegaRadiansPerSecond
