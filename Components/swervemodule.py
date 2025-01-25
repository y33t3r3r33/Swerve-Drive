import math

import phoenix6 as ctre
import rev
import wpimath
from rev._rev import *
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState


# class SwerveModule:
#     def __init__(self, dc, tc, ec):
#         self.dm = rev.Sparkmax(dc, rev.CANSparkMax.SparkLowLevel.kBrushless)
#         self.dmenc = self.dm.getEncoder(SparkRelativeEncoder.type.kHallSensor, 42)

#         # self.tm = rev.SparkMax(tc.SparkLowLevel.kBrushless)

#         self.enc = ctre.hardware.CANcoder(ec)

#         self.tkp = 2.5
#         self.tkd = 0.5

#         self.tpid = controller.PIDController(self.tkp, 0, self.tkd)
#         self.tpid.enableContinuousInput(-0.5, 0.5)
#         self.tpid.setSetpoint(0.0)

#         self.dkp = 0.001
#         self.dki = 0
#         self.dkd = 0

#         self.dpid = controller.ProfiledPIDController(self.dkp, self.dki, self.dkd,
#                                                      wpimath.trajectory.TrapezoidProfile.Constraints(3, 10))

#     def setswivdir(self):
#         pass

#     def getrotencposeasdouble(self):
#         return self.enc.get_absolute_position().value

#     def getdriveencpose(self):
#         return self.dmenc.getPosition()

#     def getSwerveModPose(self) -> SwerveModulePosition:
#         return SwerveModulePosition(
#             self.getdriveencpose(),
#             Rotation2d(self.ticks2rad(self.getrotencposeasdouble()))
#         )

#     def getabsencrad(self) -> float:
#         angle = self.getrotencposeasdouble()
#         angle *= 2 * math.pi
#         return angle

#     def getdrivevelo(self):
#         return self.dmenc.getVelocity()

#     def getState(self):
#         return SwerveModuleState(self.getdrivevelo(), Rotation2d(self.getrotencposeasdouble()))

#     def setdesstate(self, state: SwerveModuleState):
#         self.state = SwerveModuleState.optimize(state, self.getState().angle)
#         self.dm.set(self.state.speed / 3)
#         self.tm.set(
#             self.tpid.calculate(self.getabsencrad(), self.state.angle.radians()))

#     def ticks2rad(self, something):
#         """

#         :param something: ticks
#         :return: ur mom
#         """
#         return (something / .5) * -math.pi

#     def setRotPow(self, power):
#         self.tm.set(power)
