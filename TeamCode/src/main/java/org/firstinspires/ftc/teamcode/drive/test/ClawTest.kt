package org.firstinspires.ftc.teamcode.drive.test

import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ClawTest: OpMode() {
    private lateinit var left: SimpleServo
    private lateinit var right: SimpleServo
    override fun init() {
        left = SimpleServo(hardwareMap, "clawLeft", -10.0, 10.0)
        right = SimpleServo(hardwareMap, "clawLeft", -10.0, 10.0)

        left.position = 0.0
        right.position = 0.0
    }

    override fun loop() {
        if (gamepad1.a) {
            left.position = 0.85
            right.position = -0.85
        }
        if (gamepad1.b) {
            left.position = -0.85
            right.position = 0.85
        }
        if (gamepad1.x) {
            left.position = 0.0
            right.position = 0.0
        }

    }

}