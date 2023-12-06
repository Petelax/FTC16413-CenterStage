package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants.ElevatorPIDConstants.kG
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Drivebase
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile

@TeleOp(group = "test")
class ElevatorFFTest: OpMode() {
    private lateinit var elevator: Elevator
    private lateinit var wrist: Wrist
    private var setpoint = 0.0
    private var working = 1.0
    private lateinit var pid: ProfiledPIDController
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        wrist = Wrist(hardwareMap)
        val k = DrivebaseConstants.ElevatorPIDConstants
        pid = ProfiledPIDController(k.kP, k.kI, k.kD, TrapezoidProfile.Constraints(k.MAX_VELO, k.MAX_ACCEL))
        pid.setTolerance(k.POSITION_TOLERANCE, k.VELOCITY_TOLERANCE)
    }

    override fun loop() {
        wrist.set(DrivebaseConstants.WristPositions.BOTTOM)
        if (gamepad1.a) {
            working = 1.0
            setpoint = Elevator.BOTTOM
        }

        if (gamepad1.b) {
            working = 1.0
            setpoint = Elevator.INTAKE
        }

        if (gamepad1.x) {
            working = 1.0
            setpoint = Elevator.TOP - 1
        }

        if (gamepad1.right_bumper) {
            working = 1.0
            setpoint = Elevator.TOP / 2
        }

        if (gamepad1.y) {
            working = 1.0
            setpoint = Elevator.ROTATE
        }

        val currentPosition = elevator.getCurrentPosition()
        val power = pid.calculate(currentPosition, setpoint) + kG
        telemetry.addData("atSetpoint", pid.atSetpoint())
        telemetry.addData("currentPosition", currentPosition)
        telemetry.addData("setpoint", setpoint)
        telemetry.addData("power", power * working)
        elevator.set(power)
        elevator.periodic()
        telemetry.update()
    }

}