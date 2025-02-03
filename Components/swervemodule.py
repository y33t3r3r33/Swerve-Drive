import math
import time

import phoenix6 as ctre
import rev
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

DRIVE_GEAR_RATIO = 6.75
DRIVE_WHEEL_RADIUS: wpimath.units.meters = 0.0508
DRIVE_MOTOR_MAX_RPM = 5676

def encoder_to_Rotation2d(encoder_position) -> Rotation2d:
    return Rotation2d.fromDegrees(encoder_position * 360)

def degrees_to_encoder(angle) -> float:
    return wpimath.inputModulus(angle, -180, +180) / 360

def drive_motor_rev_to_distance(revolutions: float) -> wpimath.units.meters:
    wheel_rotations = wpimath.units.rotationsToRadians(revolutions / DRIVE_GEAR_RATIO)
    return wheel_rotations * DRIVE_WHEEL_RADIUS

def distance_to_drive_motor_rev(distance: wpimath.units.meters) -> float:
    wheel_rotations = distance / DRIVE_WHEEL_RADIUS
    return wpimath.units.radiansToRotations(wheel_rotations) * DRIVE_GEAR_RATIO

def drive_motor_rpm_to_percent(rpm: wpimath.units.revolutions_per_minute) -> float:
    return rpm / DRIVE_MOTOR_MAX_RPM

def velocity_to_drive_motor_percent(velocity: wpimath.units.meters_per_second) -> float:
    return clamp(drive_motor_rpm_to_percent(distance_to_drive_motor_rev(velocity) * 60), -1, 1)

def percent_to_drive_motor_rpm(percent: float) -> wpimath.units.revolutions_per_minute:
    return percent * DRIVE_MOTOR_MAX_RPM

def clamp(value, lower_bound, upper_bound):
    return min(max(value, lower_bound), upper_bound)

class SwerveModule:
    """One swerve drive pod"""

    def __init__(self,
                 drive_motor_id: int,
                 rotation_motor_id: int,
                 rotation_canbus_encoder_id: int,
                 invert_drive: bool = False) -> None:
        self.drive_motor = rev.SparkMax(drive_motor_id,
                                        rev.SparkLowLevel.MotorType.kBrushless)
        self.drive_encoder = self.drive_motor.getEncoder()

        self.max_drive_step = 0.15

        # self.drive_pid = wpimath.controller.PIDController(0.05, 0, 0.02)
        # self.drive_pid.disableContinuousInput()

        self.rotation_motor = rev.SparkMax(rotation_motor_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.rotation_encoder = ctre.hardware.CANcoder(rotation_canbus_encoder_id)

        self.rotation_pid = wpimath.controller.ProfiledPIDController(4, 0.15, 0.04,
                                                                     trajectory.TrapezoidProfile.Constraints(50, 55))
        self.rotation_pid.setIZone(0.1)
        self.rotation_pid.enableContinuousInput(-0.5, 0.5)

        self.current_position: SwerveModulePosition = SwerveModulePosition(0, Rotation2d(1, 0))

        self.target_state: SwerveModuleState = SwerveModuleState(0, Rotation2d(1, 0))

        self.invert_drive = invert_drive

        self.rotation_threshold = 0.1
        self.rotation_full_power_threshold = 0.03

        self.disabled = True

    def _update_state(self) -> None:
        drive_position = drive_motor_rev_to_distance(self.drive_encoder.getPosition())

        rotation_angle_raw = self.rotation_encoder.get_absolute_position().value_as_double
        rotation_angle = encoder_to_Rotation2d(rotation_angle_raw)

        self.current_position = SwerveModulePosition(drive_position, rotation_angle)

    def _update_rotation(self) -> None:
        """Returns angle off of target"""
        encoder_position = degrees_to_encoder(self.current_position.angle.degrees())
        target_position = degrees_to_encoder(self.target_state.angle.degrees())
        new_target = self.rotation_pid.calculate(encoder_position,
                                                 target_position)
        new_target = clamp(new_target, -1, 1)
        self.rotation_motor.set(new_target)

    def _update_drive(self) -> None:
        current_drive_percent = self.drive_motor.get()

        target_drive_percent = velocity_to_drive_motor_percent(self.target_state.speed)

        if self.invert_drive:
            target_drive_percent *= -1

        if abs(target_drive_percent - current_drive_percent) > self.max_drive_step:
            if target_drive_percent > current_drive_percent:
                target_drive_percent = current_drive_percent + self.max_drive_step
            else:
                target_drive_percent = current_drive_percent - self.max_drive_step

        power_scale = (self.target_state.angle - self.current_position.angle).cos()

        self.drive_motor.set(target_drive_percent * power_scale)

    def update(self) -> None:
        self._update_state()

        if self.disabled:
            self.rotation_motor.disable()
            self.drive_motor.disable()
            return

        self._update_rotation()
        # power_scale = 0
        # if rotation_distance_off < self.rotation_full_power_threshold:
        #     power_scale = 1
        # elif rotation_distance_off < self.rotation_threshold:
        #     power_scale = self.rotation_threshold - rotation_distance_off
        #     power_scale /= self.rotation_threshold - self.rotation_full_power_threshold

        self._update_drive()

    def set_state_from_speed_and_angle(self, speed: wpimath.units.meters_per_second, angle: Rotation2d) -> None:
        self.target_state = SwerveModuleState(speed, angle)
        self.target_state.optimize(encoder_to_Rotation2d(self.rotation_encoder.get_absolute_position().value_as_double))
        # self.calculate_rotation_pid_target(angle.degrees() / 180)

    def set_state(self, state: SwerveModuleState) -> None:
        self.set_state_from_speed_and_angle(state.speed, state.angle)

    def get_state(self) -> SwerveModuleState:
        return self.target_state

    def get_position(self) -> SwerveModulePosition:
        return self.current_position

    def disable(self):
        self.disabled = True
        self.drive_motor.disable()
        self.rotation_motor.disable()

    def enable(self):
        self.disabled = False

    def reset_distance(self):
        self.drive_encoder.setPosition(0)
        self.current_position = SwerveModulePosition(0, self.current_position.angle)
