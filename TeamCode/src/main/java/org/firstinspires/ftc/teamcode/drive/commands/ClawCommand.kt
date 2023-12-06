package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Claw
import java.util.function.DoubleSupplier

class ClawCommand(claw: Claw, gripPos: Double): CommandBase() {
    private var claw: Claw
    private var gripPos: Double
    private var startTime: Double
    private var targetTime: Double
    init {
        this.claw = claw
        this.gripPos = gripPos
        startTime = System.nanoTime() / 1E9
        targetTime = 0.023

        addRequirements(claw)
    }

    override fun initialize() {
        claw.setGripper(gripPos)
    }

    override fun execute() {
        claw.setGripper(gripPos)
    }

    override fun end(interrupted: Boolean) {
        //claw.setGripper(DrivebaseConstants.GrabberPositions.OPEN)
    }

    override fun isFinished(): Boolean {
        //return (System.nanoTime() / 1E9) - (startTime) > targetTime
        return true

    }

}