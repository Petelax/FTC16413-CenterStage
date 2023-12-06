package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.test.CommandBasedNotBasedTest
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Wrist

class ScoringCommand(elevator: Elevator, arm: Arm, wrist: Wrist, state: CommandBasedNotBasedTest.State): CommandBase() {
    private var elevator: Elevator
    private var arm: Arm
    private var wrist: Wrist
    private var state: CommandBasedNotBasedTest.State
    init {
        this.elevator = elevator
        this.arm = arm
        this.wrist = wrist
        this.state = state
    }

    override fun initialize() {

    }

    override fun execute() {
        super.execute()
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }

}