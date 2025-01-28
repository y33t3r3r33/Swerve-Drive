import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from robotcontainer import RobotContainer

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d

import Components.drivetrain
import Components.claw
import Components.arm

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
        self.claw = Components.claw.Claw()
        self.arm = Components.arm.Arm()

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
        self.slow = 1.5

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()
        
        xspeed = self.driver1.getRightX() * self.slow
        yspeed = self.driver1.getRightY() * self.slow

        rot_speed = self.driver1.getLeftX() * 2

        self.drivetrain.drive_vector(xspeed, yspeed, rot_speed)
        
        self.drivetrain.update()
        # print(self.drivetrain.odometry.getPose())

        if self.driver2.getLeftBumperButtonPressed():
            self.claw.ClawSetPower(0.3)
        else:
            self.claw.ClawSetPower(0)

        if self.driver2.getYButtonPressed():
            self.arm.ArmSwiv(0.3)
        elif self.driver2.getXButtonPressed():
            self.arm.ArmSwiv(-0.3)
        else:
            self.arm.ArmSwiv(0)

        if self.driver2.getAButtonPressed():
            self.arm.ArmExtend(0.3)
        elif self.driver2.getBButtonPressed():
            self.arm.ArmExtend(-0.3)
        else:
            self.arm.ArmExtend(0)


        # self.arm.ArmSwiv(self.driver2.RightX)
        #
        # self.arm.ArmSwiv(self.driver2.RightY)