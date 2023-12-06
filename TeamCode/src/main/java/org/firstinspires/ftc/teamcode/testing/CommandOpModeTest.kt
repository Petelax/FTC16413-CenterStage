package org.firstinspires.ftc.teamcode.testing

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "test")
class CommandOpModeTest: CommandOpMode() {
    override fun initialize() {
        val intake = Intake(hardwareMap)
        val gamepad = GamepadEx(gamepad1)

        intake.defaultCommand = IntakeCommand(intake, { gamepad.leftY })

    }

}