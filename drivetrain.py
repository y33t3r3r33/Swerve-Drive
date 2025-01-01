import math

import navx
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

def lratio(angle):
   return ((angle / math.pi) * -.5)

def ticks2rad(thing):
   return (thing / .5) * -math.pi

def deg2rot2d(deg) -> Rotation2d:
   SwerveModulePosition()
   return Rotation2d(deg.valueasdouble % 360 * (math.pi / 180))

def getSwerveModPos(rotenc: ctre.hardware.CANcoder, driveenc: rev.CANSparkMaxRelativeEncoder) -> SwerveModulePosition:
   return SwerveModulePosition(
      driveenc.getPosition(),
      Rotation2d(ticks2rad(rotenc.getabsoluteposition().valueasdouble))
   )

class Drivetrain():
   def __init__(self) -> None:
      super().__init__()

      self.robotContainer = robotcontainer

      self.backLeftRotation = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
      self.backRightRotation = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
      self.frontLeftRotation = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
      self.frontRightRotation = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)

      self.backLeftDrive = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
      self.backRightDrive = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
      self.frontLeftDrive = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
      self.frontRightDrive = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)

      self.backLeftRotation.setOpenLoopRampRate(0.2)
      self.backRightRotation.setOpenLoopRampRate(0.2)
      self.frontLeftRotation.setOpenLoopRampRate(0.2)
      self.frontRightRotation.setOpenLoopRampRate(0.2)

      self.backLeftDrive.setOpenLoopRampRate(0.2)
      self.backRightDrive.setOpenLoopRampRate(0.2)
      self.frontLeftDrive.setOpenLoopRampRate(0.2)
      self.frontRightDrive.setOpenLoopRampRate(0.2)

      self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.frontRightDriveEnc = self.frontRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.backLeftDriveEnc = self.backLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
      self.backRightDriveEnc = self.backRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

      self.BleftEnc = ctre.hardware.CANcoder(12)
      self.BrightEnc = ctre.hardware.CANcoder(11)
      self.FleftEnc = ctre.hardware.CANcoder(13)
      self.FrightEnc = ctre.hardware.CANcoder(10)

      self.lcs = ChassisSpeeds(0, 0, 0) #last chassis speed

      RotKp = 2
      RotKi = 0
      RotKd = 0.1
      self.BleftPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.BleftPID.enableContinuousInput(-0.5, 0.5)
      self.BleftPID.setSetpoint(0.0)
      self.BrightPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.BrightPID.enableContinuousInput(-0.5, 0.5)
      self.BrightPID.setSetpoint(0.0)
      self.FleftPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.FleftPID.enableContinuousInput(-0.5, 0.5)
      self.FleftPID.setSetpoint(0.0)
      self.FrightPID = controller.PIDController(RotKp, RotKi, RotKd)
      self.FrightPID.enableContinuousInput(-0.5, 0.5)
      self.FrightPID.setSetpoint(0.0)

      DriveKp = 0.01
      DriveKi = 0
      DriveKd = 0

      self.FrontRightDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                                 wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.FrontLeftDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                                wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.BackRightDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                                wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.BackLeftDrivePID = controller.ProfiledPIDController(DriveKp, DriveKi, DriveKd,
                                                               wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

      self.gyro = navx.AHRS.create_i2c(wpilib.I2C.port.kMXP)

      self.gyro.enableLogging(True)

      frontrightlocation = Translation2d(-2.08, -2.08)
      frontleftlocation = Translation2d(-2.08, 2.08)
      backleftlocation = Translation2d(2.08, 2.75)
      backrightlocation = Translation2d(2.08, -2.75)

      self.kinematics = SwerveDrive4Kinematics(
         frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
      )
      self.odometry = SwerveDrive4Odometry(
         self.kinematics,
         wpimath.geometry.Rotation2d().fromDegrees(self.gyro.getYaw())
         ,
         (
            getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
            getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
            getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
            getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
         ),
         Pose2d(0, 0, Rotation2d().fromDegrees(0))
         # starting pose 5 meters against the wall 13.5 from the driver station and a heading of 0
      )

      def shouldFlipPath(self):
         pass
         # Boolean supplier that controls when the path will be mirrored for the red alliance
         # This will flip the path being followed to the red side of the field.
         # THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      # return DriverStation.getAlliance() == DriverStation.Alliance.kRed
      def getGyro(self):
         return -self.gyro.getAngle()

      def getChassisSpeed(self) -> ChassisSpeeds:
         print(f"{self.lastChassisSpeed=}")
         return self.lastChassisSpeed

      def updateOdometry(self) -> None:
         self.odometry.update(
            wpimath.geometry.Rotation2d().fromDegrees(self.gyro.getYaw())
            ,
            (
               getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
               getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
               getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
               getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
            )
         )

      def resetOdometry(self):
         self.odometry.resetPosition(
            wpimath.geometry.Rotation2d().fromDegrees(self.gyro.getYaw())
            ,
            (
               getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
               getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
               getSwerveModPos(self.BrightEnc, self.backRightDriveEnc),
               getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc)
            ),
            Pose2d(0, 0, Rotation2d.fromDegrees(0))
         )

      def periodic(self) -> None:
         self.updateOdometry()

      def testDrive(self, speeds: ChassisSpeeds) -> None:
         print("TEST DRIVE")
         print(speeds)

      def testGetPose(self) -> Pose2d:
         print("getPOSE")
         return Pose2d()

      def angleToEncTics(self, angle: float) -> float:

         return self.scale_number(angle, 0, 360, -0.5, 0.99973)

      def ticksToAngle(self, ticks: float):
         return self.scale_number(ticks, -0.5, 0.99973, 0, 360)

      def scale_number(self, unscaled, to_min, to_max, from_min, from_max):
         """
         scales numbers using some cool math with other stuff

         """
         return (to_max - to_min) * (unscaled - from_min) / (from_max - from_min) + to_min

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
         frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

         frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
                                                         Rotation2d(
                                                            ticks2rad(self.FleftEnc.get_absolute_position()._value)))
         frontRightOptimized = SwerveModuleState.optimize(frontRight,
                                                          Rotation2d(
                                                             ticks2rad(self.FrightEnc.get_absolute_position()._value)))
         backLeftOptimized = SwerveModuleState.optimize(backLeft,
                                                        Rotation2d(
                                                           ticks2rad(self.BleftEnc.get_absolute_position()._value)))
         backRightOptimized = SwerveModuleState.optimize(backRight,
                                                         Rotation2d(
                                                            ticks2rad(self.BrightEnc.get_absolute_position()._value)))

         self.backLeftRotation.set(
            -self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value,  # was negative
                                     lratio(backLeftOptimized.angle.radians())))
         self.frontLeftRotation.set(self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value,
                                                            lratio(frontLeftOptimized.angle.radians())))
         self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value,
                                                              # was negative
                                                              lratio(backRightOptimized.angle.radians())))
         self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value,
                                                               # was negative
                                                               lratio(frontRightOptimized.angle.radians())))

         self.backLeftDrive.set(-backLeftOptimized.speed)  # was negative
         self.backRightDrive.set(backRightOptimized.speed)
         self.frontLeftDrive.set(frontLeftOptimized.speed)
         self.frontRightDrive.set(frontRightOptimized.speed)

         self.updateOdometry()

      def align(self):
         results = self.Eyes.getResults()
         if results.hasTargets():
            for i in results.getTargets():
               if i.getFiducialId() == 7 or i.getFiducialId() == 3:
                  self.yaw = i.getYaw()
               else:
                  return 0

            return self.robotRotPID.calculate(Rotation2d.fromDegrees(self.gyro.getAngle()).degrees(), self.yaw)
         else:
            return 0