import math

import rev
import wpilib
import wpimath
from wpimath import controller
from wpimath import trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

class Claw:
    def __init__(self):
        super().__init__()

        self.claw1 = rev.SparkMax(9, rev.SparkMax.MotorType.kBrushless)
        self.claw2 = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)

        self.claw1enc = self.claw1.getEncoder()
        self.claw2enc = self.claw2.getEncoder()

        ckp = 0.5
        cki = 0
        ckd = 0

        self.claw1pid = controller.PIDController(ckp, cki, ckd)
        self.claw1pid.enableContinuousInput(-0.5, 0.5)
        self.claw1pid.setSetpoint(0.0)
        self.claw2pid = controller.PIDController(ckp, cki, ckd)
        self.claw2pid.enableContinuousInput(-0.5, 0.5)
        self.claw2pid.setSetpoint(0.0)
