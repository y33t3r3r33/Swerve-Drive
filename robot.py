import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import Components.drivetrain

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d


class State():
    def __init__(self, state: str):
        self.state = state
        pass

    def changeState(self, state: str):
        self.state = state

    def getState(self):
        return self.state

class MyRobot(wpilib.TimedRobot):

    def __init__(self):
        super().__init__()
        self.driver1 = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        self.xsl = wpimath.filter.SlewRateLimiter(3)  # x speed limiter
        self.ysl = wpimath.filter.SlewRateLimiter(3)  # y rate limiter
        self.rl = wpimath.filter.SlewRateLimiter(3)  # rot limiter

        def disabledInit(self):
            pass

        def teleopInit(self):
            self.slow = 1

        def teleopPeriodic(self):
            xspeed = self.driver1.getLeftX() * 0.1
            yspeed = self.driver1.getLeftY() * 0.1

            speeds = ChassisSpeeds.fromRobotRelativeSpeeds(xspeed, yspeed, 0, Rotation2d().fromDegrees(0))
            # speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -xspeed * self.slow,
            #                                               Rotation2d().fromDegrees(
            #                                                     self.yaw))
            self.swerve.driveFromChassisSpeeds(speeds)