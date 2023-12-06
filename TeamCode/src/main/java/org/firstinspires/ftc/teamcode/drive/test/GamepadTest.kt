package org.firstinspires.ftc.teamcode.drive.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "test")
class GamepadTest: OpMode() {
    override fun init() {
        gamepad1.setLedColor(1.0, 0.0, 0.0, 2000)

    }

    override fun loop() {
        gamepad1.setLedColor(1.0, 0.0, 1.0, 500)
    }

}