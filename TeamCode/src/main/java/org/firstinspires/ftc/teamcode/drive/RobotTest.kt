package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.PI
import kotlin.math.atan2

@TeleOp
class RobotTest: OpMode() {
    private lateinit var robot: Robot

    override fun init() {
        robot = Robot(this)

    }

    override fun loop() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = -1.0*gamepad1.left_stick_y.toDouble()
        val angle: Double = (atan2(y, x) * 180.0 / PI) - 90.0
        //robot.translate(angle, gamepad1.right_stick_y.toDouble())
        robot.turnInplace(gamepad1.left_stick_x.toDouble())

        robot.status()
    }
}