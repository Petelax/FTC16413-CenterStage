package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

/**
 * This file was a mistake, but I'm too lazy to change it and call swerveDrivebase directly
 */
class Robot(opMode: OpMode) {
    private var drivebase: SwerveDrivebase

    private val telemetry = MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().telemetry)


    init {
        drivebase = SwerveDrivebase(opMode.hardwareMap)
    }

    /**
     * The only good one
     * @param speeds the x velocity, y velocity, and angular velocity of the chassis
     */
    fun kinematicsDrive(speeds: ChassisSpeeds) {
        drivebase.kinematicsDrive(speeds)
    }

    fun fieldCentricDrive(strafeSpeed: Double, forwardSpeed: Double, turnSpeed: Double) {
        drivebase.fieldCentricDrive(strafeSpeed, forwardSpeed, turnSpeed)
    }

    fun resetGyro() {
        drivebase.resetGyro()
    }

    fun getHeading(): Double {
        return drivebase.getHeading()
    }

    fun maintainHeading() {
        drivebase.maintainHeading()
    }

    fun status() {
        telemetry.addData("heading", drivebase.getHeading())
        val status = drivebase.status()
        telemetry.addLine(status)
        telemetry.update()
    }

    fun periodic() {
        drivebase.periodic()
    }

    fun getPose(): Pose2d {
        return drivebase.getPose()
    }

    fun setPose(pose2d: Pose2d) {
        drivebase.setPose(pose2d)
    }
}