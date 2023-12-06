package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.constants.Units
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrive
import org.firstinspires.ftc.teamcode.rr.TwoDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import kotlin.math.PI


@TeleOp(group = "test")
class RoadRunnerOdoTest: CommandOpMode() {
    private var pose: Pose2d = Pose2d(0.0, 0.0, 0.0)
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var localizer: TwoDeadWheelLocalizer
    private val TICKS_PER_MM = 8192 / (35 * PI)
    private val INCHES_PER_TICK = (1.377953 * PI) / 8192

    private var lastTime: Long = 0

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drivebase = SwerveDrivebase(hardwareMap)

        val imu = hardwareMap.get<IMU>(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )
        imu.initialize(parameters)

        localizer = TwoDeadWheelLocalizer(hardwareMap, imu, INCHES_PER_TICK)

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

        updatePoseEstimate()

        //val pose = drivebase.getPose()
        telemetry.addData("x", pose.position.x)
        telemetry.addData("y", pose.position.y)
        telemetry.addData("heading", pose.heading.real)
        telemetry.addData("hz ", 1E9/(currentTime-lastTime))
        telemetry.update()
        lastTime = currentTime
        super.run()
    }

    private fun updatePoseEstimate(): PoseVelocity2d {
        val twist: Twist2dDual<Time> = localizer.update()
        pose = pose.plus(twist.value())

        return twist.velocity().value()
    }
}
