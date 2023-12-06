package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.wpilib.ProfiledPIDController
import org.firstinspires.ftc.teamcode.wpilib.TrapezoidProfile
import kotlin.math.PI

@TeleOp(group = "test")
class TurningTest: OpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var headingController: ProfiledPIDController
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drivebase = SwerveDrivebase(hardwareMap)

        val c = DrivebaseConstants.DrivebasePIDConstants.Heading
        this.headingController = ProfiledPIDController(c.kP, c.kI, c.kD, TrapezoidProfile.Constraints(c.MAX_VELO, c.MAX_ACCEL))
        headingController.setTolerance(c.POSITION_TOLERANCE, c.VELOCITY_TOLERANCE)
        headingController.enableContinuousInput(-PI, PI)

        headingController.setGoal(PI/2)

    }

    override fun loop() {
        val vw = headingController.calculate(drivebase.getHeadingRad())
        drivebase.fieldCentricDrive(0.0, 0.0, vw)

        telemetry.addData("rad", drivebase.getHeadingRad())
        telemetry.update()
        drivebase.periodic()
    }
}