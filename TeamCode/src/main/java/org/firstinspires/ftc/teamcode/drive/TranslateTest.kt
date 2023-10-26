package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import kotlin.math.atan2

class TranslateTest: OpMode() {
    private lateinit var drive: SwerveDrivebase
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drive = SwerveDrivebase(hardwareMap)
    }

    override fun loop() {
        val angle = atan2(gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble())
        telemetry.addData("angle", angle)
        val power = gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble()
        drive.translate(angle, power)
        telemetry.update()
    }
}