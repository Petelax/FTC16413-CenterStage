package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile
import java.util.function.DoubleSupplier

class ArmPIDCommand(arm: Arm, setpoint: Double): CommandBase() {
    private var arm: Arm
    private var controller: ProfiledPIDController
    private var setpoint: Double
    init {
        this.arm = arm
        this.setpoint = setpoint
        val pids = DrivebaseConstants.ArmPIDConstants
        controller = ProfiledPIDController(pids.kP, pids.kI, pids.kD, TrapezoidProfile.Constraints(pids.MAX_VELO, pids.MAX_ACCEL))
        controller.setTolerance(pids.POSITION_TOLERANCE, pids.VELOCITY_TOLERANCE)

        addRequirements(arm)
    }

    override fun execute() {
        arm.set(controller.calculate(arm.getCurrentPosition(), setpoint))
    }

    override fun end(interrupted: Boolean) {
        arm.set(0.0)
    }

    override fun isFinished(): Boolean {
        return controller.atSetpoint()
    }
}