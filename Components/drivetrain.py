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

def encoder_to_Rotation2d(encoder_position) -> Rotation2d:
    return Rotation2d.fromDegrees(encoder_position * 180 + 180)


class Drivetrain():

    def __init__(self):
        super().__init__()

        #self.robotContainer = robotcontainer

        self.back_left_rotation = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.back_right_rotation = rev.SparkMax(8, rev.SparkMax.MotorType.kBrushless)
        self.front_left_rotation = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushless)
        self.front_right_rotation = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)

        self.back_left_drive = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushless)
        self.back_right_drive = rev.SparkMax(4, rev.SparkMax.MotorType.kBrushless)
        self.front_left_drive = rev.SparkMax(6, rev.SparkMax.MotorType.kBrushless)
        self.front_right_drive = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)

        self.back_left_drive_encoder = self.back_left_drive.getEncoder()
        self.back_right_drive_encoder = self.back_right_drive.getEncoder()
        self.front_left_drive_encoder = self.front_left_drive.getEncoder()
        self.front_right_drive_encoder = self.front_right_drive.getEncoder()

        self.back_left_rotation_encoder = ctre.hardware.CANcoder(12)
        self.back_right_rotation_encoder = ctre.hardware.CANcoder(11)
        self.front_left_rotation_encoder = ctre.hardware.CANcoder(13)
        self.front_right_rotation_encoder = ctre.hardware.CANcoder(10)

        self.lcs = ChassisSpeeds()

        rkp = 0.5
        rki = 0
        rkd = 0

        self.blpid = controller.PIDController(rkp, rki, rkd)
        self.blpid.enableContinuousInput(-0.5, 0.5)
        self.blpid.setSetpoint(0.0)
        self.brpid = controller.PIDController(rkp, rki, rkd)
        self.brpid.enableContinuousInput(-0.5, 0.5)
        self.brpid.setSetpoint(0.0)
        self.flpid = controller.PIDController(rkp, rki, rkd)
        self.flpid.enableContinuousInput(-0.5, 0.5)
        self.flpid.setSetpoint(0.0)
        self.frpid = controller.PIDController(rkp, rki, rkd)
        self.frpid.enableContinuousInput(-0.5, 0.5)
        self.frpid.setSetpoint(0.0)

        self.rrpid = controller.PIDController(0.001, 0, 0)

        dkp = 0.01
        dki = 0.0
        dkd = 0.0

        self.bldpid = controller.ProfiledPIDController(
            dkp, dki, dkd,
            wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
        self.brdpid = controller.ProfiledPIDController(
            dkp, dki, dkd,
            wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
        self.fldpid = controller.ProfiledPIDController(
            dkp, dki, dkd,
            wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
        self.frdpid = controller.ProfiledPIDController(
            dkp, dki, dkd,
            wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
        self.fldpid = controller.ProfiledPIDController(
            dkp, dki, dkd,
            wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

        self.gyro = navx.AHRS.create_spi()

        self.gyro.enableLogging(True)

        self.bll_pos = Translation2d(.5, .5)
        self.brl_pos = Translation2d(.5, -.5)
        self.fll_pos = Translation2d(-.5, -.5)
        self.frl_pos = Translation2d(-.5, .5)

        self.kinematics = SwerveDrive4Kinematics(self.fll_pos, self.frl_pos, self.bll_pos, self.brl_pos)

        print("End of init")

    def getAutoComm(self):
        print("getautcommand")

    def shouldFlipPath(self):
        pass

    def getGyro(self):
        return -self.gyro.getAngle()

    def getChassisSpeed(self) -> ChassisSpeeds:
        print(f"{self.lcs=}")
        return self.lcs

    def updateodo(self) -> None:
        pass

    def periodic(self) -> None:
        pass
        # self.updateodo()

    def testDrive(self, speeds: ChassisSpeeds):
        print("TEST DRIVE MODE")
        print(speeds)

    def testGetPose(self) -> Pose2d:
        print("TEST GET POSE")
        return Pose2d()

    def angleToEncTics(self, angle: float):
        return self.scale_number(angle, 0, 360, -0.5, 0.99973)

    def scale_number(self, unscaled, to_min, to_max, from_min, from_max):
        """
      scales numbers using some cool math with other stuff

      """
        return (to_max - to_min) * (unscaled - from_min) / (from_max -
                                                            from_min) + to_min

    # def TurnSwivelPos(self):

    #     newAngle = self.angleToEncTics(45)

    #     FLnewState = self.optimize(0, newAngle,
    #                                self.front_left_rotation_encoder.get_absolute_position().value)
    #     FRnewState = self.optimize(0, 360 - newAngle,
    #                                self.front_right_rotation_encoder.get_absolute_position().value)
    #     BLnewState = self.optimize(0, 360 - newAngle,
    #                                self.back_left_rotation_encoder.get_absolute_position().value)
    #     BRnewState = self.optimize(0, newAngle,
    #                                self.back_right_rotation_encoder.get_absolute_position().value)

    #     FLnewSteerAngle = FLnewState[1]
    #     FRnewSteerAngle = FRnewState[1]
    #     BLnewSteerAngle = BLnewState[1]
    #     BRnewSteerAngle = BRnewState[1]

    #     FLOutput = self.flpid.calculate(
    #         self.front_left_rotation_encoder.get_absolute_position().value, FLnewSteerAngle)
    #     FROutput = self.frpid.calculate(
    #         self.front_right_rotation_encoder.get_absolute_position().value, FRnewSteerAngle)
    #     BLOutput = self.blpid.calculate(
    #         self.back_left_rotation_encoder.get_absolute_position().value, BLnewSteerAngle)
    #     BROutput = self.brpid.calculate(
    #         self.back_right_rotation_encoder.get_absolute_position().value, BRnewSteerAngle)

    #     self.front_left_rotation.set(FLOutput)
    #     self.front_right_rotation.set(FROutput)
    #     self.back_left_rotation.set(BLOutput)
    #     self.back_right_rotation.set(BROutput)

    # def optimize(self, drive_voltage, steer_angle, current_angle):
    #     delta = steer_angle - current_angle

    #     if abs(delta) > math.pi / 2.0 and abs(delta) < 3.0 / 2.0 * math.pi:
    #         if steer_angle >= math.pi:
    #             return (-drive_voltage, steer_angle - math.pi)
    #         else:
    #             return (-drive_voltage, steer_angle + math.pi)
    #     else:
    #         return (drive_voltage, steer_angle)

    def drive_from_vector_and_angle(self, x, y, rot) -> None:
        self.driveFromChassisSpeeds(ChassisSpeeds(x, y, rot))
    
    def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        self.lastChassisSpeed = speeds

        fl, fr, bl, br = self.kinematics.toSwerveModuleStates(speeds)

        fl.optimize(encoder_to_Rotation2d(self.front_left_rotation_encoder.get_absolute_position().value_as_double))
        fr.optimize(encoder_to_Rotation2d(self.front_right_rotation_encoder.get_absolute_position().value_as_double))
        bl.optimize(encoder_to_Rotation2d(self.back_left_rotation_encoder.get_absolute_position().value_as_double))
        br.optimize(encoder_to_Rotation2d(self.back_right_rotation_encoder.get_absolute_position().value_as_double))

        print(f"{fl.angle} {fr.angle} {bl.angle} {br.angle}")

    # def align(self):
    #    results = self.Eyes.getResults()
    #    if results.hasTargets():
    #       for i in results.getTargets():
    #          if i.getFiducialId() == 7 or i.getFiducialId() == 3:
    #             self.yaw = i.getYaw()
    #          else:
    #             return 0
    #
    #       return self.robotRotPID.calculate(Rotation2d.fromDegrees(self.gyro.getAngle()).degrees(), self.yaw)
    #    else:
