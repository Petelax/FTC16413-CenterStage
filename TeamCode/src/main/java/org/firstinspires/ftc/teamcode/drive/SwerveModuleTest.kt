package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule

@TeleOp
class SwerveModuleTest: OpMode() {
    private lateinit var LF: SwerveModule
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        LF = SwerveModule(hardwareMap, DrivebaseConstants.DeviceIDs.RR_DRIVE_MOTOR, DrivebaseConstants.DeviceIDs.RR_TURN_MOTOR, DrivebaseConstants.DeviceIDs.RR_ENCODER)
        LF.enabled = true
        LF.override = true
    }

    override fun loop() {
        val left: Double = gamepad1.left_stick_y.toDouble()
        val right: Double = gamepad1.right_stick_y.toDouble()

        LF.setOverrideTurnSpeed(left)
        LF.setSpeed(right)
        //LF.setAbsoluteAngle(0.0)
        telemetry.addLine("RR")
        telemetry.addData("desiredAngle", LF.getDesiredAngle())
        telemetry.addData("currentAngle", LF.getAngle())
        telemetry.addData("turnPower", LF.getTurnPower())
        telemetry.addData("drivePower", LF.getSpeed())
        LF.update()
        telemetry.update()
    }
}