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
import Components.vision
import Components.claw
import Components.arm
import Components.elevator

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
        self.elevator = Components.elevator.Elevator()

        self.state = State("disabled")

        self.xsl = wpimath.filter.SlewRateLimiter(3)  # x speed limiter
        self.ysl = wpimath.filter.SlewRateLimiter(3)  # y rate limiter
        self.rl = wpimath.filter.SlewRateLimiter(3)  # rot limiter

        self.position_test = False

        self.repositioning = False

        self.field_relative_drive = True

        self.rotation_track_test = Rotation2d(1, 0)

        self.autonomous_state = 0
        self.autonomous_in_flight = False  # Make sure we don't accidentally stage immediately after startup
        self.autonomous_coords = [(-4, -1, Rotation2d(-1, 0)),
                                  (-4,  2, Rotation2d(-1, 0)),
                                  (-2,  2, Rotation2d(-1, 0))]
        print(self.autonomous_coords)

        self.vision = Components.vision.Vision()

    def disabledInit(self):
        self.drivetrain.stop()
        self.drivetrain.disable()
        self.claw.Disable()
        self.claw.Stop()
        self.elevator.Disable()
        self.elevator.Stop()
        self.arm.Disable()
        self.arm.Stop()

    def disabledExit(self):
        self.drivetrain.reset()
        self.drivetrain.enable()

    def autonomousInit(self):
        self.drivetrain.set_robot_location(-2, -1, Rotation2d(-1, 0))
        self.autonomous_state = 0

    def autonomousPeriodic(self):
        # Make sure we hit the target coordinate
        if self.autonomous_in_flight and not self.drivetrain.arrived_at_target():
            return

        if self.autonomous_state >= len(self.autonomous_coords):
            self.drivetrain.stop()
            if self.autonomous_in_flight:
                print("Done")
            self.autonomous_in_flight = False
            return

        self.autonomous_in_flight = True
        xpos, ypos, heading = self.autonomous_coords[self.autonomous_state]
        self.drivetrain.drive_vector_position(xpos, ypos, heading)
        print(f"Running stage {self.autonomous_state}...")
        self.autonomous_state += 1

    def robot(self):
        pass
        # self.robotcontainer = RobotContainer()
        # self.drivetrain = self.robotcontainer.drivetrain

    def robotPeriodic(self):
        self.vision.poll()
        self.drivetrain.update()
        self.arm.Update()
        self.claw.Update()
        self.elevator.Update()

    def teleopInit(self):
        self.slow = 4
        # self.drivetrain.set_robot_location(-3, 0, Rotation2d(-1, 0))

    def teleopPeriodic(self):
        # self.robotcontainer = RobotContainer()

        if self.repositioning and self.drivetrain.arrived_at_target():
            self.repositioning = False
            self.position_test = False
            print("We've arrived!")
        
        if self.driver1.getAButtonPressed():
            self.position_test = True
            self.rotation_track_test = Rotation2d(1, 0)
        if self.driver1.getBButtonPressed():
            self.position_test = False
            self.repositioning = False

        if self.driver1.getYButtonPressed():
            # self.drivetrain.set_wheel_angles(Rotation2d(1, 0))
            self.drivetrain.reset()
            return

        if self.driver1.getXButton():
            self.drivetrain.stop()
            return

        xspeed = self.driver1.getRightX() * self.slow
        yspeed = self.driver1.getRightY() * self.slow

        # print(xspeed)
        # print(yspeed)

        rot_speed = self.driver1.getLeftX() * math.pi

        
        if self.position_test:
            # print(self.drivetrain.odometry.getPose())
            if not self.repositioning:
                self.drivetrain.drive_vector_position(0, 0, Rotation2d(1, 0))
                self.repositioning = True
        else:
            if self.field_relative_drive:
                self.drivetrain.drive_vector_velocity_field_relative(-yspeed, -xspeed, -rot_speed)
            else:
                self.drivetrain.drive_vector_velocity(-yspeed, -xspeed, -rot_speed)

        # print(self.drivetrain.odometry.getPose())

        if self.driver2.getLeftBumperButton():
            self.claw.ClawSetPower(1)
        elif self.driver2.getRightBumperButton():
            self.claw.ClawSetPower(-1)
        else:
            self.claw.ClawSetPower(0)

            if self.driver2.getYButtonPressed() and self.driver2.getRightStickButton():
                self.arm.ArmSwiv(0.3)
            elif self.driver2.getXButtonPressed() and self.driver2.getRightStickButton():
                self.arm.ArmSwiv(-0.3)
            else:
                self.arm.ArmSwiv(0)

            if self.driver2.getAButtonPressed() and self.driver2.getRightStickButton():
                self.arm.ArmExtend(0.3)
            elif self.driver2.getBButtonPressed() and self.driver2.getRightStickButton():
                self.arm.ArmExtend(-0.3)
            else:
                self.arm.ArmExtend(0)

            if self.driver2.getAButton() and self.elevator.getLimit2() == True:
                self.elevator.EleExtend(1)

            if self.driver2.getBButton() and self.elevator.getLimit3() == True:
                self.elevator.EleExtend(1)

            if self.driver2.getXButton() and self.elevator.getLimit3() == True:
               self.elevator.EleExtend(1)

            if self.driver2.getYButton() and self.elevator.getLimit1() == True:
               self.elevator.EleExtend(-1)

        self.claw.WristMove(self.driver2.getRightX() * 0.2)

        if self.driver2.getRightStickButton():
            print(self.claw.Update())
            print(self.arm.Update())
            print(self.elevator.Update())
            print(self.drivetrain.update())