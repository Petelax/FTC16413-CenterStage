package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import java.util.function.DoubleSupplier

class WristCommand(wrist: Wrist, position: Double): CommandBase() {
    private var wrist: Wrist
    private var position: Double
    init {
        this.wrist = wrist
        this.position = position

        addRequirements(wrist)
    }

    override fun execute() {
        wrist.set(position)
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return true
    }
}