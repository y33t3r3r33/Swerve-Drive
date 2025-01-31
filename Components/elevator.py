import rev

class Elevator():
        def __init__(self):
               self.kracken = rev.SparkMax(19, rev.SparkMax.MotorType.kBrushless)

        def EleExtend(self, power):
                self.kracken.set(power)