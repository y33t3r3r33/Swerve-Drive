import wpilib

class Limit():
    def __init__(self):
        self.limit1 = wpilib.DigitalInput(0)
        self.limit2 = wpilib.DigitalInput(1)
        self.limit3 = wpilib.DigitalInput(2)
        self.limit4 = wpilib.DigitalInput(3)
#        self.limit5 = wpilib.DigitalInput(4)
    
    def getLimit1(self):
        self.limit1.get()
   
    def getLimit2(self):
        self.limit2.get()
    
    def getLimit3(self):
        self.limit3.get()
   
    def getLimit4(self):
        self.limit4.get()

#    def getLimit5(self):
#        self.limit5.get()