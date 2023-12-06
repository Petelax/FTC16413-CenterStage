package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrive
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase

@TeleOp(group = "test")
class OdometryTest: CommandOpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private var lastTime: Long = 0

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drivebase = SwerveDrivebase(hardwareMap)

        val driverOp = GamepadEx(gamepad1)

        drivebase.defaultCommand = FieldCentricDrive(
            drivebase,
            {driverOp.leftY},
            {driverOp.leftX*-1.0},
            {driverOp.rightX*-1.0},
            {driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)},
            { true }
        )
    }

    override fun run() {
        val currentTime = System.nanoTime()
        val pose = drivebase.getPose()
        telemetry.addData("x", pose.x)
        telemetry.addData("y", pose.y)
        telemetry.addData("heading", pose.heading)
        telemetry.addData("hz ", 1E9/(currentTime-lastTime))
        telemetry.update()
        lastTime = currentTime
        super.run()
    }
}