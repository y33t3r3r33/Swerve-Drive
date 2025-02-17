import rev

class Arm():
	def __init__(self):
		self.armswiv1 = rev.SparkMax(16, rev.SparkMax.MotorType.kBrushless)
		self.armswiv2 = rev.SparkMax(17, rev.SparkMax.MotorType.kBrushless)
		self.armextend = rev.SparkMax(18, rev.SparkMax.MotorType.kBrushed)

	def ArmSwiv(self, power):
		self.armswiv1.set(power)
		self.armswiv2.set(power)

	def ArmExtend(self, power):
		self.armextend.set(power)

	def Disable(self):
		self.armswiv1.disable()
		self.armswiv2.disable()
		self.armextend.disable()

	def Update(self):
		self.armswiv1.getEncoder()
		self.armswiv2.getEncoder()
		self.armextend.getEncoder()

	def Stop(self):
		self.armswiv1.set(0)
		self.armswiv2.set(0)
		self.armextend.set(0)