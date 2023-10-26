package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
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

    fun translate(angle: Double, speed: Double) {
        drivebase.translate(angle, speed)

    }

    fun turnInplace(speed: Double) {
        drivebase.turnInplace(speed)
    }

    fun drive(angle: Double, translateSpeed: Double, turnSpeed: Double) {
        drivebase.translateTurn(angle, translateSpeed, turnSpeed)
    }

    /**
     * The only good one
     * @param speeds the x velocity, y velocity, and angular velocity of the chassis
     */
    fun kinematicsDrive(speeds: ChassisSpeeds) {
        drivebase.kinematicsDrive(speeds)
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