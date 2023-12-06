package org.firstinspires.ftc.teamcode.drive.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase

@TeleOp(group = "test")
class WtfTest: OpMode() {
    private lateinit var drivebase: SwerveDrivebase
    override fun init() {
        drivebase = SwerveDrivebase(hardwareMap)

    }

    override fun loop() {
        telemetry.addData("pose", drivebase.getPose())
        telemetry.update()
    }
}