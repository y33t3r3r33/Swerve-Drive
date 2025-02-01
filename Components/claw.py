import phoenix6
import rev

class Claw():
    def __init__(self):
        self.claw1 = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)
        self.claw2 = rev.SparkMax(15, rev.SparkMax.MotorType.kBrushless)
        self.wrist = phoenix6.hardware.TalonFX(21, "rio")
    def ClawSetPower(self, power):
        self.claw1.set(power)
        self.claw2.set(power)

    def WristMove(self, power):
        self.wrist.set(power)