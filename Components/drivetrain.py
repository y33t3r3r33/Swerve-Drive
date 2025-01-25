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

      dkp = 0.01
      dki = 0.0
      dkd = 0.0

      self.bldpid = controller.ProfiledPIDController(dkp, dki, dkd,
                                              wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.brdpid = controller.ProfiledPIDController(dkp, dki, dkd,
                                              wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.fldpid = controller.ProfiledPIDController(dkp, dki, dkd,
                                              wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.frdpid = controller.ProfiledPIDController(dkp, dki, dkd,
                                              wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))
      self.fldpid = controller.ProfiledPIDController(dkp, dki, dkd,
                                              wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.gyro = navx.AHRS.create_spi()

      self.gyro.enableLogging(True)

      bll = Translation2d(.5, .5)
      brl = Translation2d(.5, -.5)
      fll = Translation2d(-.5, -.5)
      frl = Translation2d(-.5, .5)

      self.kinematics = SwerveDrive4Kinematics(
         fll, frl, bll, brl
      )

      self.odo = SwerveDrive4Odometry(
         self.kinematics,
         wpimath.geometry.Rotation2d().fromDegrees(self.gyro.getYaw())
         ,
         (
            getswervemodpos(self.frenc, self.frdenc),
            getswervemodpos(self.flenc, self.fldenc),
            getswervemodpos(self.brenc, self.brdenc),
            getswervemodpos(self.blenc, self.bldenc)
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

   def scale_number(self, unscaled, to_min, to_max, from_min, from_max):
      """
      scales numbers using some cool math with other stuff

      """
      return (to_max - to_min) * (unscaled - from_min) / (from_max - from_min) + to_min

   def TurnSwivelPos(self):

      newAngle = self.angleToEncTics(45)

      FLnewState = self.optimize(0, newAngle, self.flenc.get_absolute_position().value)
      FRnewState = self.optimize(0, 360 - newAngle, self.frenc.get_absolute_position().value)
      BLnewState = self.optimize(0, 360 - newAngle, self.blenc.get_absolute_position().value)
      BRnewState = self.optimize(0, newAngle, self.brenc.get_absolute_position().value)

      FLnewSteerAngle = FLnewState[1]
      FRnewSteerAngle = FRnewState[1]
      BLnewSteerAngle = BLnewState[1]
      BRnewSteerAngle = BRnewState[1]

      FLOutput = self.flpid.calculate(self.flenc.get_absolute_position().value, FLnewSteerAngle)
      FROutput = self.frpid.calculate(self.frenc.get_absolute_position().value, FRnewSteerAngle)
      BLOutput = self.blpid.calculate(self.blenc.get_absolute_position().value, BLnewSteerAngle)
      BROutput = self.brpid.calculate(self.brenc.get_absolute_position().value, BRnewSteerAngle)

      self.flr.set(FLOutput)
      self.frr.set(FROutput)
      self.blr.set(BLOutput)
      self.brr.set(BROutput)

   def optimize(self, drive_voltage, steer_angle, current_angle):
      delta = steer_angle - current_angle

      if abs(delta) > math.pi / 2.0 and abs(delta) < 3.0 / 2.0 * math.pi:
         if steer_angle >= math.pi:
            return (-drive_voltage, steer_angle - math.pi)
         else:
            return (-drive_voltage, steer_angle + math.pi)
      else:
         return (drive_voltage, steer_angle)

   def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
      self.lastChassisSpeed = speeds
      fl, fr, bl, br = self.kinematics.toSwerveModuleStates(speeds)

      frontLeftOptimized = SwerveModuleState.optimize(fl,
                                                      Rotation2d(
                                                         ticks2rad(self.flenc.get_absolute_position().value)))
      frontRightOptimized = SwerveModuleState.optimize(fr,
                                                       Rotation2d(
                                                          ticks2rad(self.frenc.get_absolute_position().value)))
      backLeftOptimized = SwerveModuleState.optimize(bl,
                                                     Rotation2d(
                                                        ticks2rad(self.blenc.get_absolute_position().value)))
      backRightOptimized = SwerveModuleState.optimize(br,
                                                      Rotation2d(
                                                         ticks2rad(self.brenc.get_absolute_position().value)))

      self.blr.set(-self.blpid.calculate(self.blenc.get_absolute_position().value,
                                                         lrat(backLeftOptimized.angle.radians())))
      self.flr.set(self.flpid.calculate(self.flenc.get_absolute_position().value,
                                                         lrat(frontLeftOptimized.angle.radians())))
      self.brr.set(-self.brpid.calculate(self.brenc.get_absolute_position().value,
                                                           lrat(backRightOptimized.angle.radians())))
      self.frr.set(-self.frpid.calculate(self.frenc.get_absolute_position().value,
                                                            lrat(frontRightOptimized.angle.radians())))

      self.bld.set(-backLeftOptimized.speed)
      self.brd.set(backRightOptimized.speed)
      self.fld.set(frontLeftOptimized.speed)
      self.frd.set(frontRightOptimized.speed)

      self.updateodo()

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
   #       return 0