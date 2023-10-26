package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder

@TeleOp
class EncoderTest: OpMode() {
    private var lf: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    private var rf: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    private var lr: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    private var rr: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()

    private lateinit var driveLF: Motor
    private lateinit var driveRF: Motor
    private lateinit var driveLR: Motor
    private lateinit var driveRR: Motor

    private lateinit var voltageSensor: VoltageSensor
    override fun init() {
        voltageSensor = hardwareMap.get(VoltageSensor::class.java, "Control Hub")
        val IDs = DrivebaseConstants.DeviceIDs

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        lf.init(hardwareMap, DrivebaseConstants.DeviceIDs.LF_ENCODER)
        rf.init(hardwareMap, DrivebaseConstants.DeviceIDs.RF_ENCODER)
        lr.init(hardwareMap, DrivebaseConstants.DeviceIDs.LR_ENCODER)
        rr.init(hardwareMap, DrivebaseConstants.DeviceIDs.RR_ENCODER)

        driveLF = Motor(hardwareMap, IDs.LF_DRIVE_MOTOR)
        driveRF = Motor(hardwareMap, IDs.RF_DRIVE_MOTOR)
        driveLR = Motor(hardwareMap, IDs.LR_DRIVE_MOTOR)
        driveRR = Motor(hardwareMap, IDs.RR_DRIVE_MOTOR)

        lf.setOffset(DrivebaseConstants.Measurements.LF_OFFSET)
        rf.setOffset(DrivebaseConstants.Measurements.RF_OFFSET)
        lr.setOffset(DrivebaseConstants.Measurements.LR_OFFSET)
        rr.setOffset(DrivebaseConstants.Measurements.RR_OFFSET)
    }

    override fun loop() {
        telemetry.addData("voltage", voltageSensor.voltage)

        telemetry.addData("lf", lf.getVoltage())
        telemetry.addData("rf", rf.getVoltage())
        telemetry.addData("lr", lr.getVoltage())
        telemetry.addData("rr", rr.getVoltage())

        telemetry.addData("lf", lf.getAngle())
        telemetry.addData("rf", rf.getAngle())
        telemetry.addData("lr", lr.getAngle())
        telemetry.addData("rr", rr.getAngle())

        if (gamepad1.a) {
            driveLF.set(1.0)
            driveRF.set(1.0)
            driveLR.set(1.0)
            driveRR.set(1.0)
        } else {
            driveLF.set(0.0)
            driveRF.set(0.0)
            driveLR.set(0.0)
            driveRR.set(0.0)
        }

        telemetry.update()
    }
}