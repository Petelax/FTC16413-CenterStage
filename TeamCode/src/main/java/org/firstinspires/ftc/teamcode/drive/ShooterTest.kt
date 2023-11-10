package org.firstinspires.ftc.teamcode.drive

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.eventloop.opmode.OpMode

class ShooterTest: OpMode() {
    private lateinit var shooter: CRServo
    override fun init() {
        shooter = CRServo(hardwareMap, "shooter")
    }

    override fun loop() {
        shooter.set(if (gamepad1.a) { 1.0 } else { 0.0 } + if (gamepad1.b) { -1.0 } else { 0.0 })
    }
}