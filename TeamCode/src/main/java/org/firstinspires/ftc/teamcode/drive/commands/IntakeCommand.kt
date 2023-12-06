package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Intake
import java.util.function.DoubleSupplier


class IntakeCommand(intake: Intake, speed: DoubleSupplier): CommandBase() {
    private var intake: Intake
    private var speed: DoubleSupplier

    init {
        this.intake = intake
        this.speed = speed

        addRequirements(intake)
    }
    override fun execute() {
        intake.set(speed.asDouble)
    }

    override fun end(interrupted: Boolean) {
        intake.set(0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }
}