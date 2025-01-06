import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.DriveTrain()

        self.xsl = wpimath.filter.SlewRateLimiter(3) #x speed limiter
        self.ysl = wpimath.filter.SlewRateLimiter(3) #y rate limiter
        self.rl = wpimath.filter.SlewRateLimiter(3) #rot limiter

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool):
        xSpeed = (
            -self.xsl.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        ySpeed = (
            -self.ysl.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        rot = (
            -self.rl.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())