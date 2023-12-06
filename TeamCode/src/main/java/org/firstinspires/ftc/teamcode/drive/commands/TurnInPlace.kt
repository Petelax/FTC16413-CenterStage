package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.sign

class TurnInPlace(drivebase: SwerveDrivebase, rotation: Rotation2d, voltage: DoubleSupplier): CommandBase() {
    private var drivebase: SwerveDrivebase
    private var rotation: Rotation2d
    private var headingController: ProfiledPIDController

    init {
        this.drivebase = drivebase
        this.rotation = rotation

        val c = DrivebaseConstants.DrivebasePIDConstants.Heading
        this.headingController = ProfiledPIDController(c.kP, c.kI, c.kD, TrapezoidProfile.Constraints(c.MAX_VELO, c.MAX_ACCEL))
        headingController.setTolerance(c.POSITION_TOLERANCE, c.VELOCITY_TOLERANCE)
        headingController.enableContinuousInput(-PI, PI)

    }

    override fun initialize() {
        headingController.setGoal(rotation.radians)
    }

    override fun execute() {
        var vw = headingController.calculate(drivebase.getHeadingRad())
        vw += vw.sign * DrivebaseConstants.DrivebasePIDConstants.kF

        drivebase.fieldCentricDrive(0.0, 0.0, vw)
    }


    override fun end(interrupted: Boolean) {
        val pose = drivebase.getPose()
        drivebase.maintainHeading()

    }

    override fun isFinished(): Boolean {
        return headingController.atSetpoint()
    }


}
