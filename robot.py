import wpilib
import wpilib.drive
import wpilib.shuffleboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds

from robotcontainer import RobotContainer

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
        self.joystick = wpilib.Joystick(0)
        board = wpilib.shuffleboard.Shuffleboard
        self.state = State('Disabled')

    def disabledPeriodic(self):
        pass

    def robotInit(self):

        self.robotContainer = RobotContainer()
        self.drivetrain = self.robotContainer.drivetrain

        self.drivetrain.gyro.zeroYaw()

    def teleopInit(self):
        self.drivetrain.gyro.zeroYaw()

        self.slow = 1

    def teleopPeriodic(self):
        xspeed = self.driver1.getX()
        yspeed = self.driver1.getY()

        if self.driver1.getRawButtonPressed(2):
            self.drivetrain.gyro.zeroYaw()

        if self.driver1.getTrigger():
            tspeed = self.driver1.getZ()
        else:
            tspeed = 0

        if xspeed == 0 and yspeed == 0 and tspeed == 0:
            self.drivetrain.frontLeftDrive.set(0)
            self.drivetrain.backRightDrive.set(0)
            self.drivetrain.backLeftDrive.set(0)
            self.drivetrain.frontRightDrive.set(0)

            self.drivetrain.backLeftRotation.set(0)
            self.drivetrain.backRightRotation.set(0)
            self.drivetrain.frontLeftRotation.set(0)
            self.drivetrain.frontRightRotation.set(0)

        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(yspeed * self.slow, -xspeed * self.slow, tspeed * 0.8,
                                                       Rotation2d().fromDegrees(
                                                           self.yaw))
        self.drivetrain.driveFromChassisSpeeds(speeds)

        print(self.drivetrain.odometry.getPose())

    def testInit(self):
        pass

    def testPeriodic(self):
        print(self.drivetrain.odomerty.getPose())
        pass

    def robotPeriodic(self):
        self.yaw = -self.drivetrain.gyro.getAngle()

if __name__ == "__main__":
    wpilib.run(MyRobot)