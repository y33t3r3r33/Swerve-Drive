import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
# from robotcontainer import RobotContainer

import rev

from Components import claw

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

    def robotInit(self) -> None:
        super().__init__()
        # self.robotcontainer = RobotContainer()
        self.driver1 = wpilib.XboxController(0)
        self.driver2 = wpilib.XboxController(1)
        # self.swerve = drivetrain.Drivetrain()

        self.state = State("disabled")

        self.claw1 = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)
        self.claw2 = rev.SparkMax(15, rev.SparkMax.MotorType.kBrushless)

        self.xsl = wpimath.filter.SlewRateLimiter(3)  # x speed limiter
        self.ysl = wpimath.filter.SlewRateLimiter(3)  # y rate limiter
        self.rl = wpimath.filter.SlewRateLimiter(3)  # rot limiter

    def disable(self):
        pass

    # def robot(self):
        # self.robotcontainer = RobotContainer()
        # self.drivetrain = self.robotcontainer.drivetrain

        # self.blr = self.robotcontainer.drivetrain.blr
        # self.brr = self.robotcontainer.drivetrain.brr
        # self.flr = self.robotcontainer.drivetrain.flr
        # self.frr = self.robotcontainer.drivetrain.frr
        #
        # self.blpid = self.robotcontainer.drivetrain.blpid
        # self.brpid = self.robotcontainer.drivetrain.brpid
        # self.flpid = self.robotcontainer.drivetrain.flpid
        # self.frpid = self.robotcontainer.drivetrain.frpid
        #
        # self.blenc = self.robotcontainer.drivetrain.blenc
        # self.brenc = self.robotcontainer.drivetrain.brenc
        # self.flenc = self.robotcontainer.drivetrain.flenc
        # self.frenc = self.robotcontainer.drivetrain.frenc
        #
        # self.drivetrain.gyro.zeroYaw()

    def teleopInit(self):
        self.slow = 1

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()
        # self.drivetrain = self.robotcontainer.drivetrain()
        # self.claw1 = self.robotcontainer.claw.claw1
        # self.claw2 = self.robotcontainer.claw.claw2
        # self.claw1 = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)
        # self.claw2 = rev.SparkMax(15, rev.SparkMax.MotorType.kBrushless)
        xspeed = self.driver1.getRightX()
        yspeed = self.driver1.getRightY()
        tspeed = self.driver1.getRightX()

        clawspeed1 = self.driver2.getRightY()

        # if self.driver1.B():
        #     self.drivetrain.gyro.zeroYaw()
        # else:
        #     tspeed = 0

        # if abs(xspeed) < .15:  # applies  a deadzone to the joystick
        #     xspeed = 0
        # if abs(yspeed) < .2:
        #     yspeed = 0

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



        # speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -xspeed * self.slow, -tspeed * 0.8,
        #                                                Rotation2d().fromDegrees(
        #                                                    self.drivetrain.getGyro()))
        #
        # self.drivetrain.driveFromChassisSpeeds(speeds)

        self.claw1.set(-clawspeed1 * 0.3)
        self.claw2.set(clawspeed1 * 0.3)


        # if self.driver1.getLeftBumperButtonPressed() == True:
        #     self.claw1.set(-yeetspeed)
        #     self.claw2.set(yeetspeed)
        #
        # if self.driver1.getRightBumperButtonPressed() == True:
        #     self.claw1.set(0)
        #     self.claw2.set(0)

        # print(self.drivetrain.odometry.getPose())