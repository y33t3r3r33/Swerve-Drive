import math

import phoenix6 as ctre
import rev
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

def encoder_to_Rotation2d(encoder_position) -> Rotation2d:
    return Rotation2d.fromDegrees(encoder_position * 180)

def degrees_to_encoder(angle) -> float:
    return wpimath.inputModulus(angle, -180, +180) / 360

class SwerveModule:
    """One swerve drive pod"""

    def __init__(self,
                 drive_motor_id: int,
                 rotation_motor_id: int,
                 rotation_canbus_encoder_id: int,
                 invert_target: bool = False,
                 invert_angle: bool = False,
                 invert_drive: bool = False) -> None:
        self.drive_motor = rev.SparkMax(drive_motor_id,
                                        rev.SparkLowLevel.MotorType.kBrushless)
        self.drive_encoder = self.drive_motor.getEncoder()

        self.rotation_motor = rev.SparkMax(rotation_motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.rotation_encoder = ctre.hardware.CANcoder(rotation_canbus_encoder_id)

        self.rotation_pid = wpimath.controller.PIDController(1.8, 0.5, 0)

        self.target_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))

        self.cross_sign = False
        self.target_cross_sign = 0
        self.old_encoder = 0

        self.invert_motor = invert_target
        self.invert_angle = invert_angle
        self.invert_drive = invert_drive

        self.rotation_threshold = 0.04

    def update(self) -> None:
        self.target_state.optimize(encoder_to_Rotation2d(self.rotation_encoder.get_absolute_position().value_as_double))
        encoder_position = self.rotation_encoder.get_absolute_position().value_as_double
        target_position = degrees_to_encoder(self.target_state.angle.degrees())
        if self.cross_sign:
            if abs(self.old_encoder) > encoder_position:
                self.cross_sign = False
            else:
                target_position = self.target_cross_sign

        new_target = self.rotation_pid.calculate(self.rotation_encoder.get_absolute_position().value_as_double,
                                                 target_position)

        if self.invert_motor:
            new_target *= -1

        self.rotation_motor.set(new_target)

        drive_speed = min(abs(self.target_state.speed), 2) / 2

        if self.target_state.speed < 0:
            drive_speed *= -1
            
        if self.invert_drive:
            drive_speed *= -1

        if abs(target_position - encoder_position) < self.rotation_threshold:
            self.drive_motor.set(drive_speed)
        else:
            self.drive_motor.set(0)
        
    def set_state_from_speed_and_angle(self, speed: wpimath.units.meters_per_second, angle: Rotation2d) -> None:
        # angle /= 2
        if self.invert_angle:
            angle *= -1
        self.target_state = SwerveModuleState(speed, angle)
        # self.calculate_rotation_pid_target(angle.degrees() / 180)

    def calculate_rotation_pid_target(self, target_value) -> None:
        self.target_cross_sign = self.rotation_encoder.get_absolute_position().value_as_double - target_value
        if abs(self.target_cross_sign) > 1:
            self.cross_sign = True

    def set_state(self, state: SwerveModuleState) -> None:
        self.set_state_from_speed_and_angle(state.speed, state.angle)
