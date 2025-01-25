import math

import navx
# from wpilib import DriverStation
import phoenix6 as ctre
import rev
import wpilib
import wpimath
from navx import AHRS
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, \
   SwerveModulePosition

from .swervemodule import SwerveModule

def lrat(angle):
    return ((angle / math.pi) * -.5)


def ticks2rad(michlsoft):
    return (michlsoft / .5) * math.pi


def deg2rot2d(deg) -> Rotation2d:
    SwerveModulePosition()
    return Rotation2d(deg.value_as_double % 360 * (math.pi / 180))


def getswervemodpos(rotenc: ctre.hardware.CANcoder,
                    denc: rev) -> SwerveModulePosition:
    return SwerveModulePosition(
        denc.getPosition(),
        Rotation2d(ticks2rad(rotenc.get_absolute_position().value)))

class Drivetrain():

    def __init__(self):
        self.front_left = SwerveModule(6, 3, 13, False, True, True)
        self.front_right = SwerveModule(5, 2, 10, False, False, True)
        self.back_left = SwerveModule(7, 1, 12, True, True, False)
        self.back_right = SwerveModule(4, 8, 11, True, True, True)

        self.gyro = navx.AHRS.create_spi()

        self.gyro.enableLogging(True)

        self.bll_pos = Translation2d(.5, .5)
        self.brl_pos = Translation2d(.5, -.5)
        self.fll_pos = Translation2d(-.5, .5)
        self.frl_pos = Translation2d(-.5, -.5)

        self.kinematics = SwerveDrive4Kinematics(self.fll_pos, self.frl_pos, self.bll_pos, self.brl_pos)

        self.test_target_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))

        print("End of init")

    def update(self):
        self.front_left.update()
        self.front_right.update()
        self.back_left.update()
        self.back_right.update()

    def set_wheel_angles(self, angle):
        self.front_left.set_state_from_speed_and_angle(0, angle)
        self.front_right.set_state_from_speed_and_angle(0, angle)
        self.back_left.set_state_from_speed_and_angle(0, angle)
        self.back_right.set_state_from_speed_and_angle(0, angle)

    def set_wheel_speed_and_angle(self, speed, angle):
        self.front_left.set_state_from_speed_and_angle(speed, angle)
        self.front_right.set_state_from_speed_and_angle(speed, angle)
        self.back_left.set_state_from_speed_and_angle(speed, angle)
        self.back_right.set_state_from_speed_and_angle(speed, angle)

    def drive_vector(self, x, y, rot):
        chassis_speeds = ChassisSpeeds(x, y, rot)

        fl_state, fr_state, bl_state, br_state = self.kinematics.toSwerveModuleStates(chassis_speeds, Translation2d(0, 0))

        self.front_left.set_state(fl_state)
        self.front_right.set_state(fr_state)
        self.back_left.set_state(bl_state)
        self.back_right.set_state(br_state)
