package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Elevator

@TeleOp
class ElevatorTest: OpMode() {
    private lateinit var elevator: Elevator

    //private lateinit var elevator: MotorGroup
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
    }

    override fun loop() {
        if (gamepad1.a) {
            elevator.setpoint = Elevator.TOP
        }
        if (gamepad1.b) {
            elevator.setpoint = Elevator.BOTTOM
        }

        elevator.set(-1.0*gamepad1.left_stick_y.toDouble())

        elevator.update()
        telemetry.addData("elevator", elevator.getCurrentPosition())
        telemetry.addData("speed", elevator.get())
        telemetry.update()
    }
}