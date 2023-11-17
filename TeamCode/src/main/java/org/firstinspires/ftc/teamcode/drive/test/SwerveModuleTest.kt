package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule
import kotlin.math.PI
import kotlin.math.atan2

@TeleOp
class SwerveModuleTest: OpMode() {
    private lateinit var LF: SwerveModule
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        LF = SwerveModule(hardwareMap, DrivebaseConstants.DeviceIDs.LF_DRIVE_MOTOR, DrivebaseConstants.DeviceIDs.LF_TURN_MOTOR, DrivebaseConstants.DeviceIDs.LF_ENCODER)
    }

    override fun loop() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = -1.0*gamepad1.left_stick_y.toDouble()
        val angle: Double = (atan2(y, x) * 180.0 / PI)


        LF.setAngle(angle)
        LF.setSpeed(gamepad1.right_stick_y.toDouble())
        LF.update()
        //LF.setAbsoluteAngle(0.0)
        telemetry.addLine("LF")
        telemetry.addData("sentAngle", "%.3f".format(angle))
        telemetry.addData("desiredAngle", "%.3f".format(LF.getDesiredAngle()))
        telemetry.addData("currentAngle", "%.3f".format(LF.getAngle()))
        telemetry.addData("turnPower", "%.3f".format(LF.getTurnPower()))
        telemetry.addData("drivePower", "%.3f".format(LF.getSpeed()))
        telemetry.update()
    }
}