package org.firstinspires.ftc.teamcode.drive.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.abs
import kotlin.math.hypot

@TeleOp
class FieldCentricTeleOp: OpMode() {
    private lateinit var robot: Robot
    private lateinit var elevator: Elevator

    override fun init() {
        robot = Robot(this)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)

    }

    override fun loop() {
        val x = 1.0*gamepad1.left_stick_x.toDouble()
        val y = 1.0*gamepad1.left_stick_y.toDouble()
        //val angle: Double = (atan2(y, x) * 180.0 / PI) - 90.0
        val magnitude: Double = hypot(x, y)
        val turn = gamepad1.right_stick_x.toDouble()

        if (magnitude<0.001 && abs(turn)<0.001) {
            robot.maintainHeading()
        } else {
            robot.fieldCentricDrive(y, x, turn)
        }

        if (gamepad1.a) {
            robot.resetGyro()
        }

        elevator.set(gamepad1.right_trigger.toDouble() - gamepad1.left_trigger.toDouble())

        telemetry.addData("ele pos", elevator.getCurrentPosition())
        telemetry.addData("ele speed", elevator.get())
        telemetry.addData("x", "%.2f".format(x))
        telemetry.addData("y", "%.2f".format(y))
        robot.status()
        elevator.update()
        robot.periodic()
    }
}