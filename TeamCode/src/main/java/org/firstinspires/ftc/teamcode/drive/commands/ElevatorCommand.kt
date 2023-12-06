package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import java.util.function.DoubleSupplier

class ElevatorCommand(elevator: Elevator, speed: DoubleSupplier): CommandBase() {
    private var elevator: Elevator
    private var speed: DoubleSupplier

    init {
        this.elevator = elevator
        this.speed = speed

        addRequirements(elevator)
    }

    override fun initialize() {
        elevator.mode = Elevator.Mode.RAW
    }

    override fun execute() {
        elevator.mode = Elevator.Mode.RAW
        elevator.set(speed.asDouble)
    }

    override fun end(interrupted: Boolean) {
        //elevator.set(0.0)
        //elevator.mode = Elevator.Mode.IDLE
    }

    override fun isFinished(): Boolean {
        return false
    }
}