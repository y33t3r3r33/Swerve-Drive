import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.driver1 = wpilib.XboxController(0)
        self.swerve = drivetrain.DriveTrain()

        self.xsl = wpimath.filter.SlewRateLimiter(3) #x speed limiter
        self.ysl = wpimath.filter.SlewRateLimiter(3) #y rate limiter
        self.rl = wpimath.filter.SlewRateLimiter(3) #rot limiter


    def disabledInit(self):
        pass

    def teleopInit(self):
        self.slow = 1

    def teleopPeriodic(self):
        xspeed = self.driver1.getRightX()
        yspeed = self.driver1.getRightY()

        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -xspeed * self.slow,
                                                       Rotation2d().fromDegrees(
                                                           self.yaw))