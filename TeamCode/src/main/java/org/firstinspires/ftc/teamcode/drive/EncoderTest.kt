package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder

@TeleOp
class EncoderTest: OpMode() {
    private var encoder: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        encoder.init(hardwareMap, DrivebaseConstants.DeviceIDs.LF_ENCODER)
    }

    override fun loop() {
        telemetry.addData("angle", encoder.getAngle())
        telemetry.addData("voltage", encoder.getVoltage())
        telemetry.update()
    }
}