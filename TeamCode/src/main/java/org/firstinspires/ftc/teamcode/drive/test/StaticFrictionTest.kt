package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServoImplEx
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

@TeleOp(group = "test")
class StaticFrictionTest: OpMode() {
    private lateinit var turn: CRServoImplEx
    private var voltage = 0.01
    private var feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.123, 0.001, 0.0)

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        turn = hardwareMap.get(CRServoImplEx::class.java, DrivebaseConstants.DeviceIDs.LF_TURN_MOTOR)
    }

    override fun loop() {
        voltage += gamepad1.left_stick_y.toDouble() * 0.0005
        turn.power = voltage

        telemetry.addData("voltage", voltage)
        // voltage to start moving is 0.123
        // voltage to surpass static friction is 0.120

    }
}