package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorCommand
import org.firstinspires.ftc.teamcode.drive.commands.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.subsystems.Elevator

@TeleOp(group = "test")
class ElevatorCommandTest: CommandOpMode() {
    private lateinit var elevator: Elevator
    private lateinit var gamepad: GamepadEx
    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        gamepad = GamepadEx(gamepad1)

        /*
        schedule(
            SequentialCommandGroup(
                ElevatorPIDCommand(elevator, Elevator.TOP/2),
                ElevatorPIDCommand(elevator, Elevator.ROTATE)
            )
        )
         */

        /*
        elevator.defaultCommand = ElevatorCommand(elevator) {
            gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(
                GamepadKeys.Trigger.LEFT_TRIGGER
            )
        }

         */

    }

    override fun run() {

        elevator.set(gamepad.leftY * -1.0)
        telemetry.update()
        super.run()
    }
}