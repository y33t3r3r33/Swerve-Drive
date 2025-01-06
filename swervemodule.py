import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

kwr = 0.0508 #kWheelRadius
ker = 4096 #kEncoderResolution
kmmav = math.pi #kModuleAngularVelocity
kmmaa = math.tau #kModuleMaxAngularAcceleration

class SwerevModule:
    def __init__(
            self,
            driveMotorChannel: int,
            turningMotorChannel: int,
            driveEncoderChannelA:int,
            driveEncoderChannelB: int,
            turningEncoderChannelA: int,
            turningEncoderChannelB: int,
    ) -> None:
        self.dm = wpilib.PWMSparkMax(driveMotorChannel)
        self.tm = wpilib.PWMSparkMax(turningMotorChannel)

        self.de = wpilib.Encoder(driveEncoderChannelA, driveEncoderChannelB)
        self.te = wpilib.Encoder(turningEncoderChannelA, turningEncoderChannelB)

        self.drivePIDController = wpimath.controller.ProfilePIDControler(
            1,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constrains(
                kmmav,
                kmmaa,
            ),
        )

        self.driveFeedforware = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        self.driveEncoder.setDistancePerPulse(math.tau * kwr / ker)

        self.turningEncoder.setDistancePerPulse(math.tau / ker)

        self.turningPIDController.enableContinuoudInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        return wpimath.kinematics.SwerveModulestate(
            self.driveEncoder.getDistance(),
            wpimath.geometry.rotation2d(self.turningEncoder.getDistance()),
        )

    def SetDesiredState(
            self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        desiredState.optimeize(encoderRotation)

        desiredState.cosineScale(encoderRotation)

        driveOutput = self.drivePIDController.calulate(
            self.driveEncoder.getRate(), desiredState.speed
        )

        driveFeedforward = self.driveFeedForward.calculate(desiredState.speed)

        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), desiredState.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.dm.setVoltage(driveOutput + driveFeedforward)
        self.tm.setVoltage(turnOutput + turnFeedforward)