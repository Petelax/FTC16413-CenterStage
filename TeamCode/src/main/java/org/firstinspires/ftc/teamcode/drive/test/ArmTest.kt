package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Arm

@TeleOp(group = "test")
class ArmTest: OpMode() {
    private lateinit var arm: Arm
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        arm = Arm(hardwareMap)
    }

    override fun loop() {
        arm.set(gamepad1.left_stick_y.toDouble())

        telemetry.addData("arm", arm.getCurrentPosition())
        telemetry.update()
        arm.periodic()
    }
}