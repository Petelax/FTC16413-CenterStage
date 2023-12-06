package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Arm
import java.util.function.DoubleSupplier

class ArmCommand(arm: Arm, speed: DoubleSupplier): CommandBase() {
    private var arm: Arm
    private var speed: DoubleSupplier
    init {
        this.arm = arm
        this.speed = speed

        addRequirements(arm)
    }

    override fun execute() {
        arm.set(speed.asDouble)
    }

    override fun end(interrupted: Boolean) {
        arm.set(0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }
}