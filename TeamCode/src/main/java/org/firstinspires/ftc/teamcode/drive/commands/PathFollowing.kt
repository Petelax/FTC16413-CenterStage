package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics
import com.arcrobotics.ftclib.trajectory.Trajectory
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.wpilib.HolonomicDriveController
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile
import org.firstinspires.ftc.teamcode.wpilib.WpilibPIDController
import kotlin.math.PI

class PathFollowing(drivebase: SwerveDrivebase, pose2d: Pose2d): CommandBase() {
    private var drivebase: SwerveDrivebase
    private var controller: HolonomicDriveController
    private var xController: WpilibPIDController
    private var yController: WpilibPIDController
    private var headingController: ProfiledPIDController
    private var desiredPose: Pose2d
    private var startTime: Long
    private lateinit var trajectory: Trajectory
    private var kinematics = SwerveDriveKinematics(
        DrivebaseConstants.Measurements.LF_POS,
        DrivebaseConstants.Measurements.RF_POS,
        DrivebaseConstants.Measurements.LR_POS,
        DrivebaseConstants.Measurements.RR_POS
    )

    init {
        this.drivebase = drivebase
        val c = DrivebaseConstants.DrivebasePIDConstants
        val h = DrivebaseConstants.DrivebasePIDConstants.Heading
        xController = WpilibPIDController(c.kP, c.kI, c.kD)
        xController.setTolerance(c.POSITION_TOLERANCE, c.VELOCITY_TOLERANCE)
        yController = WpilibPIDController(c.kP, c.kI, c.kD)
        yController.setTolerance(c.POSITION_TOLERANCE, c.VELOCITY_TOLERANCE)
        headingController = ProfiledPIDController(h.kP, h.kI, h.kD, TrapezoidProfile.Constraints(h.MAX_VELO, h.MAX_ACCEL))
        headingController.enableContinuousInput(-PI, PI)
        headingController.setTolerance(h.POSITION_TOLERANCE, h.VELOCITY_TOLERANCE)
        controller = HolonomicDriveController(
            xController,
            yController,
            headingController
        )

        this.desiredPose = pose2d

        startTime = System.nanoTime()

    }

    override fun initialize() {
        var startPose = drivebase.getPose()

        val trajectoryConfig = TrajectoryConfig(
            DrivebaseConstants.Measurements.MAX_VELOCITY_IN,
            //DrivebaseConstants.Measurements.MAX_ACCEL_IN,
            1000.0

        ).setKinematics(kinematics)
        trajectory = TrajectoryGenerator.generateTrajectory(
            startPose, listOf(), desiredPose, trajectoryConfig
        )

        startTime = System.nanoTime()

    }

    override fun execute() {
        val timeElapsed = (System.nanoTime() - startTime) / 1E9
        val speeds = controller.calculate(drivebase.getPose(), trajectory.sample(timeElapsed), desiredPose.rotation)

        drivebase.kinematicsDrive(speeds)

    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        val pose = drivebase.getPose()

        val t = DrivebaseConstants.DrivebasePIDConstants.POSITION_TOLERANCE
        val s = DrivebaseConstants.DrivebasePIDConstants.Heading.POSITION_TOLERANCE

        return MathUtil.isNear(desiredPose.x, pose.x, t) &&
                MathUtil.isNear(desiredPose.y, pose.y, t) &&
                MathUtil.isNear(desiredPose.y, pose.y, s)

    }


}