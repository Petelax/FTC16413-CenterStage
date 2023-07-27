package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class Robot(opMode: OpMode) {
    private var drivebase: SwerveDrivebase

    private val telemetry = MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().telemetry)

    var enabled = true
        set(value) {
            field = value
            drivebase.enabled = value
        }

    init {
        drivebase = SwerveDrivebase(opMode.hardwareMap)
    }

    /*
    fun test(gamepad1: Gamepad) {
        val power: Double = gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", power)

        if (gamepad1.x) { swerveLF.turnPower = power; telemetry.addLine("servoLF") } else { swerveLF.turnPower = 0.0 }
        if (gamepad1.y) { swerveRF.turnPower = power; telemetry.addLine("servoRF") } else { swerveRF.turnPower = 0.0 }
        if (gamepad1.a) { swerveLR.turnPower = power; telemetry.addLine("servoLR") } else { swerveLR.turnPower = 0.0 }
        if (gamepad1.b) { swerveRR.turnPower = power; telemetry.addLine("servoRR") } else { swerveRR.turnPower = 0.0 }

        if (gamepad1.dpad_up) { swerveLF.drivePower = power; telemetry.addLine("motorLF") } else { swerveLF.drivePower = 0.0 }
        if (gamepad1.dpad_left) { swerveRF.drivePower = power; telemetry.addLine("motorRF") } else { swerveRF.drivePower = 0.0 }
        if (gamepad1.dpad_down) { swerveLR.drivePower = power; telemetry.addLine("motorLR") } else { swerveLR.drivePower = 0.0 }
        if (gamepad1.dpad_right) { swerveRR.drivePower = power; telemetry.addLine("motorRR") } else { swerveRR.drivePower = 0.0 }

    }
     */

    fun drive(angle: Double, speed: Double) {
        drivebase.translate(angle, speed)

    }

    fun spin(speed: Double) {
        drivebase.spin(speed)
    }

    fun spinAbsolute(angle: Double) {
        drivebase.spinAbsolute(angle)
    }

    fun status() {
        telemetry.addData("heading", drivebase.getHeading())
        val status = drivebase.status()
        telemetry.addLine(status)
        telemetry.update()
    }

    fun reset() {
        drivebase.reset()
    }

    fun update() {
        drivebase.update()
    }
}