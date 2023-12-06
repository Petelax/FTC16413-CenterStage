package org.firstinspires.ftc.teamcode.drive.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Elevator

class ElevatorNoVeloCommand(elevator: Elevator, setpoint: Double): CommandBase() {
    private var elevator: Elevator
    private var setpoint: Double
    private var up = true
    //private var pid: ProfiledPIDController
    init {
        this.elevator = elevator
        this.setpoint = setpoint
        //val k = DrivebaseConstants.ElevatorPIDConstants
        //pid = ProfiledPIDController(k.kP, k.kI, k.kD, TrapezoidProfile.Constraints(k.MAX_VELO, k.MAX_ACCEL))
        elevator.mode = Elevator.Mode.PID

        addRequirements(elevator)
    }

    override fun initialize() {
        elevator.mode = Elevator.Mode.PID
        elevator.velocityControl(false)

        elevator.setSetPoint(setpoint)
        //pid.setTolerance(0.125)
    }
    override fun execute() {
        //val power = pid.calculate(elevator.getCurrentPosition(), setpoint) + DrivebaseConstants.ElevatorPIDConstants.kG
        //elevator.set(power)
        elevator.setSetPoint(setpoint)
        elevator.mode = Elevator.Mode.PID
    }

    override fun end(interrupted: Boolean) {
        //elevator.mode = Elevator.Mode.IDLE

    }

    override fun isFinished(): Boolean {
        return elevator.atSetPoint()
    }


}
