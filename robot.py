import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from robotcontainer import RobotContainer

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d

import Components.drivetrain


class State():
    def __init__(self, state: str):
        self.state = state
        pass

    def changeState(self, state: str):
        self.state = state

    def getState(self):
        return self.state

class MyRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        super().__init__()
        self.robotcontainer = RobotContainer()
        self.driver1 = wpilib.XboxController(0)
        self.driver2 = wpilib.XboxController(1)
        self.drivetrain = Components.drivetrain.Drivetrain()

        self.state = State("disabled")

        self.xsl = wpimath.filter.SlewRateLimiter(3)  # x speed limiter
        self.ysl = wpimath.filter.SlewRateLimiter(3)  # y rate limiter
        self.rl = wpimath.filter.SlewRateLimiter(3)  # rot limiter

    def disable(self):
        pass

    def robot(self):
        pass
        # self.robotcontainer = RobotContainer()
        # self.drivetrain = self.robotcontainer.drivetrain

    def teleopInit(self):
        self.slow = 0.05

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()
        
        xspeed = self.driver1.getRightX() * self.slow
        yspeed = self.driver1.getRightY() * self.slow
        # tspeed = self.driver1.getRightX()

        # if self.driver1.B():
        #     self.drivetrain.gyro.zeroYaw()
        # else:
        #     tspeed = 0

        # if self.driver1.A():
        #     tspeed = self.driver1.getLeftX()
        # else:
        #     tspeed = 0

        # if xspeed == 0 and yspeed == 0 and tspeed == 0:  # if no speed is given to the motors there will be no power in any of the motors
        #     self.drivetrain.fld.set(0)
        #     self.drivetrain.brd.set(0)
        #     self.drivetrain.bld.set(0)
        #     self.drivetrain.frd.set(0)
        #
        #     self.drivetrain.blr.set(0)
        #     self.drivetrain.brr.set(0)
        #     self.drivetrain.flr.set(0)
        #     self.drivetrain.frr.set(0)

        self.drivetrain.drive_from_vector_and_angle(xspeed, yspeed, 0)

        # print(self.drivetrain.odometry.getPose())
