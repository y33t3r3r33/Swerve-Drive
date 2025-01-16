import math

import navx
# from wpilib import DriverStation
import phoenix6 as ctre
import rev
import wpilib
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, \
   SwerveModulePosition

import robotcontainer

def lrat(angle):
   return ((angle / math.pi) * -.5)

def ticks2rad(michlsoft):
   return (michlsoft / .5) * math.pi

def deg2rot2d(deg) -> Rotation2d:
   SwerveModulePosition()
   return Rotation2d(deg.value_as_double % 360 * (math.pi / 180))

def getswervemodpos(rotenc: ctre.hardware.CANcoder, denc : rev) ->SwerveModulePosition:
   return SwerveModulePosition(
      denc.getPosition(),
      Rotation2d(ticks2rad(rotenc.get_absolute_position().value))
   )

class Drivetrain():
   def __init__(self):
      super().__init__()

      self.robotContainer = robotcontainer

      self.blr = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
      self.brr = rev.SparkMax(8, rev.SparkMax.MotorType.kBrushless)
      self.flr = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushless)
      self.frr = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)

      self.bld = rev.SparkMax(7, rev.SparkMax.MotorType.kBrushless)
      self.brd = rev.SparkMax(4, rev.SparkMax.MotorType.kBrushless)
      self.fld = rev.SparkMax(6, rev.SparkMax.MotorType.kBrushless)
      self.frd = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushless)

      self.bldenc = self.bld.getEncoder()
      self.brdenc = self.brr.getEncoder()
      self.fldenc = self.flr.getEncoder()
      self.frdenc = self.frr.getEncoder()

      self.blenc = ctre.hardware.CANcoder(12)
      self.brenc = ctre.hardware.CANcoder(11)
      self.flenc = ctre.hardware.CANcoder(13)
      self.frenc = ctre.hardware.CANcoder(10)

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

      dkp = 0.005
      dki = 0.01
      dkd = 0.01

      self.bldpid = controller.PIDController(dkp, dki, dkd,
                                             wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.brdpid = controller.PIDController(dkp, dki, dkd,
                                             wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.fldpid = controller.PIDController(dkp, dki, dkd,
                                             wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.frdpid = controller.PIDController(dkp, dki, dkd,
                                             wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.fldpid = controller.PIDController(dkp, dki, dkd,
                                             wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.gyro = navx.AHRS.create_spi(wpilib.kI2C.port.kMXP)

      self.gyro.enableLogging(True)

      bll = Translation2d(.5, .5)
      brl = Translation2d(.5, -.5)
      fll = Translation2d(-.5, -.5)
      frl = Translation2d(-.5, .5)

      self.kinematics = SwerveDrive4Odometry()(
         self.kinematics,
         wpimath.geometry.Rotation2d().fromDegrees(self.gyro.getYaw())
         ,
         (
            getswervesodpos(self.blenc, self.bldenc),
            getswervepodpos(self.brenc, self.brdenc),
            getswervemodpos(self.flenc, self.fldenc),
            getswervemodpos(self.frenc, self.frdenc),
         ),
         Pose2d(0, 0, Rotation2d().fromDegrees(0))
      )

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
      self.odo.update(
         wpimath.geometry.Rotation2d(self.gyro.getYaw())
         ,
         (
            getswervemodpos(self.blenc, self.bldenc),
            getswervemodpos(self.brenc, self.brdenc),
            getswervemodpos(self.flenc, self.fldenc),
            getswervemodpos(self.frenc, self.frdenc),
         )
      )

   def periodic(self) -> None:
      self.updateodo()

   def testDrive(self, speeds: ChassisSpeeds):
      print("TEST DRIVE MODE")
      print(speeds)

   def testGetPose(self) -> Pose2d:
      print("TEST GET POSE")
      return Pose2d()

   def angleToEncTics(self, angle: float):
      return self.scale_number(angle, 0, 360, -0.5, 0.99973)

   def scale_number(self, unscaled, tomin, tomax, frommin, frommax):
      return (tomax - tomin) * (unscaled - frommin) / (frommax - frommin) + tomin