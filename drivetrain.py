import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import swervemodule

kMaxSpeed = 3.0
kMAS = math.pi #kMaxAngleSpeed

class Drivetrain:

    def __init__(self) -> None:

        #Locations
        self.fll = wpimath.geometry.Translation2d(0.381, 0.381) #temp values
        self.frl = wpimath.geometry.Translation2d(0.381, -0.381) #temp values
        self.bll = wpimath.geometry.Translation2d(-0.381, 0.381) #temp values
        self.brl = wpimath.geometry.Translation2d(-0.381, -0.381) #temp values

        #Motors
        self.fl = swervemodule.SwerveModule(1, 2, 0, 1, 2, 3)
        self.fr = swervemodule.SwerveModule(3, 4, 4, 5, 6, 7)
        self.bl = swervemodule.SwerveModule(5, 6, 8, 9, 10, 11)
        self.br = swervemodule.SwerveModule(7, 8, 12, 13, 14, 15)

        self.gyro = wpilib.AnalogGyro(0)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.fll,
            self.frl,
            self.bll,
            self.brl,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.fl.getPosition(),
                self.fr.getPosition(),
                self.bl.getPosition(),
                self.br.getPosition(),
            ),
        )

        self.gyro.reset()

        def drive(
                self,
                xSpeed: float,
                ySpeed: float,
                rot:float,
                fieldRelative: bool,
                periodSeconds: float,
        ) -> None:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                wpimath.kinematics.ChassisSpeeds.discretize(
                    (
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
            ),
            periodSeconds,
        )
    )
            wpimath.kinematics.SwerveDrive4Kinematics.desaturateWhelSpeeds(
                swerveModuleStates, kMaxSpeed
            )
            self.fl.setDesiredState(swerveModuleStates[0])
            self.fr.setDesiredState(swerveModuleStates[1])
            self.bl.setDesiredStates(swerveModuleStates[2])
            self.br.setDesiredStates(swerveModuleStates[3])

    def updateOdomerty(self) -> None:
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.fl.getPosition(),
                self.fr.getPosition(),
                self.bl.getPosition(),
                self.br.getPosition(),
            ),
        )