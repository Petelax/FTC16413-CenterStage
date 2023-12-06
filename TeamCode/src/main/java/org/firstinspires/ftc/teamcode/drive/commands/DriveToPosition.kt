package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.geometry.Pose2d
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.sign

class DriveToPosition(drivebase: SwerveDrivebase, pose: Pose2d, voltage: DoubleSupplier): CommandBase() {
    private var drivebase: SwerveDrivebase
    private var pose: Pose2d
    private var xController: ProfiledPIDController
    private var yController: ProfiledPIDController
    private var headingController: ProfiledPIDController

    init {
        this.drivebase = drivebase
        this.pose = pose

        val pids = DrivebaseConstants.DrivebasePIDConstants
        this.xController = ProfiledPIDController(pids.kP, pids.kI, pids.kD, TrapezoidProfile.Constraints(pids.MAX_VELO, pids.MAX_ACCEL))
        xController.setTolerance(pids.POSITION_TOLERANCE, pids.VELOCITY_TOLERANCE)
        this.yController = ProfiledPIDController(pids.kP, pids.kI, pids.kD, TrapezoidProfile.Constraints(pids.MAX_VELO, pids.MAX_ACCEL))
        yController.setTolerance(pids.POSITION_TOLERANCE, pids.VELOCITY_TOLERANCE)
        val c = DrivebaseConstants.DrivebasePIDConstants.Heading
        this.headingController = ProfiledPIDController(c.kP, c.kI, c.kD, TrapezoidProfile.Constraints(c.MAX_VELO, c.MAX_ACCEL))
        headingController.setTolerance(c.POSITION_TOLERANCE, c.VELOCITY_TOLERANCE)
        headingController.enableContinuousInput(-PI, PI)

    }

    override fun initialize() {
        xController.setGoal(pose.x)
        yController.setGoal(pose.y)
        headingController.setGoal(pose.rotation.radians)
    }

    override fun execute() {
        val pose = drivebase.getPose()
        var vx = xController.calculate(pose.x)
        vx += vx.sign * DrivebaseConstants.DrivebasePIDConstants.kF
        var vy = yController.calculate(pose.y)
        vy += vy.sign * DrivebaseConstants.DrivebasePIDConstants.kF
        var vw = headingController.calculate(pose.rotation.radians)
        vw += vw.sign * DrivebaseConstants.DrivebasePIDConstants.kF

        drivebase.fieldCentricDrive(vx, vy, vw)
    }


    override fun end(interrupted: Boolean) {
        val pose = drivebase.getPose()
        drivebase.maintainHeading()
        //drivebase.fieldCentricDrive(0.0, 0.0, pose.heading)

    }

    override fun isFinished(): Boolean {
        return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint()
    }


}