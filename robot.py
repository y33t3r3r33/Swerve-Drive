import math

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

        self.position_test = False

        self.rotation_track_test = Rotation2d(1, 0)

    def disabledInit(self):
        self.drivetrain.disable()

    def disabledExit(self):
        self.drivetrain.reset()
        self.drivetrain.enable()

    def robot(self):
        pass
        # self.robotcontainer = RobotContainer()
        # self.drivetrain = self.robotcontainer.drivetrain

    def robotPeriodic(self):
        self.drivetrain.update()

    def teleopInit(self):
        self.slow = 4

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()

        if self.driver1.getAButtonPressed():
            self.position_test = True
            self.rotation_track_test = Rotation2d(1, 0)
        if self.driver1.getBButtonPressed():
            self.position_test = False

        if self.driver1.getYButtonPressed():
            # self.drivetrain.set_wheel_angles(Rotation2d(1, 0))
            self.drivetrain.reset()
            return
        
        xspeed = self.driver1.getRightX() * self.slow
        yspeed = self.driver1.getRightY() * self.slow

        # print(xspeed)
        # print(yspeed)

        rot_speed = self.driver1.getLeftX() * math.pi

        if self.position_test:
            if self.driver1.getXButton():
                self.drivetrain.drive_vector_position(1, 0, Rotation2d(0, 1))
            else:
                self.drivetrain.drive_vector_position(0, 0, Rotation2d(1, 0))
        else:
            self.drivetrain.drive_vector_velocity(-yspeed, -xspeed, -rot_speed)

        # print(self.drivetrain.odometry.getPose())
