package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

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

    fun status() {
        telemetry.addData("heading", drivebase.getHeading())
        val status = drivebase.status()
        telemetry.addLine(status)
        telemetry.update()
    }
}