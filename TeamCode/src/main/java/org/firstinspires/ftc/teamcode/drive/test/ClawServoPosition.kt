package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Claw

@TeleOp(group = "test")
class ClawServoPosition: OpMode() {
    private lateinit var claw: Claw
    private var magnitude: Double = 0.0
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        claw = Claw(hardwareMap)
    }

    override fun loop() {
        if (gamepad1.a) {
            magnitude += 0.01
        }

        if (gamepad1.b) {
            magnitude -= 0.01
        }

        claw.setGripper(magnitude)
        telemetry.addData("magnitude", magnitude)
        telemetry.update()
    }
}