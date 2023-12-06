package org.firstinspires.ftc.teamcode.drive.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import kotlin.math.abs
import kotlin.math.hypot

@TeleOp
class FieldCentricTeleOp: OpMode() {
    private lateinit var robot: Robot
    private lateinit var elevator: Elevator
    private lateinit var intake: Intake
    private lateinit var arm: Arm
    private lateinit var claw: Claw
    private lateinit var wrist: Wrist

    override fun init() {
        robot = Robot(this)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        intake = Intake(hardwareMap)
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        wrist = Wrist(hardwareMap)
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

        if (gamepad1.left_stick_button) {
            robot.resetGyro()
        }

        elevator.set(gamepad2.right_trigger.toDouble() - gamepad2.left_trigger.toDouble())
        if (gamepad2.options) {
            elevator.setpoint = Elevator.INTAKE
            elevator.mode = Elevator.Mode.PID
        }
        if (gamepad2.share) {
            elevator.setpoint = Elevator.INTAKE
            elevator.mode = Elevator.Mode.RAW
        }

        intake.set(if (gamepad2.a) { 1.0 } else { 0.0 } + if (gamepad2.b) { -1.0 } else { 0.0 })

        // arm.set(if (gamepad1.dpad_up) { 0.6 } else { 0.0 } + if (gamepad1.dpad_down) { -0.6 } else { 0.0 })
        arm.set(gamepad2.left_stick_y.toDouble())

        if (gamepad2.dpad_up) {
            wrist.set(DrivebaseConstants.WristPositions.TOP)
        }
        if (gamepad2.dpad_down) {
            wrist.set(DrivebaseConstants.WristPositions.BOTTOM)
        }

        if (gamepad2.x) {
            claw.setGripper(DrivebaseConstants.GrabberPositions.OPEN)

        }
        if (gamepad2.y) {
            claw.setGripper(DrivebaseConstants.GrabberPositions.CLOSED)
        }
        /*
        val clawPos = claw.getGrabber() + gamepad2.right_stick_x.toDouble()*0.05
        claw.setGripper(clawPos)
        telemetry.addData("gripper", clawPos)
         */

        gamepad2.setLedColor(255.0, 0.0, 255.0, 5000)
        val blink = Gamepad.LedEffect.Builder().addStep(0.0, 0.0, 0.0, 500).addStep(255.0, 255.0, 255.0, 500).setRepeating(true).build()
        gamepad2.runLedEffect(blink)

        if (elevator.getRumble()) {
            gamepad2.rumble(100)
        }


        telemetry.addData("touchpad finger 1 x", gamepad1.touchpad_finger_1_x)
        telemetry.addData("touchpad finger 1 y", gamepad1.touchpad_finger_1_y)
        telemetry.addData("touchpad finger 1", gamepad1.touchpad_finger_1)

        telemetry.addData("arm", arm.getCurrentPosition())
        telemetry.addData("ele pos", elevator.getCurrentPosition())
        telemetry.addData("ele speed", elevator.get())
        telemetry.addData("x", "%.2f".format(x))
        telemetry.addData("y", "%.2f".format(y))
        robot.status()
        elevator.periodic()
        intake.periodic()
        arm.periodic()
        robot.periodic()
    }
}