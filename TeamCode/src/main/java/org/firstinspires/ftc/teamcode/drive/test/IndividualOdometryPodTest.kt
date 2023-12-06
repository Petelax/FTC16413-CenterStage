package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Twist2d
import com.arcrobotics.ftclib.hardware.RevIMU
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrive
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import kotlin.math.PI

@TeleOp(group = "test")
class IndividualOdometryPodTest: OpMode() {
    private lateinit var para: Motor.Encoder
    private lateinit var perp: Motor.Encoder
    private lateinit var gyro: RevIMU
    //private lateinit var imu: IMU
    //private lateinit var localizer: TwoDeadWheelLocalizer
    //private val TICKS_PER_INCH = 100.0/245832.0
    //private lateinit var odometry: HolonomicOdometry
    private val TICKS_PER_MM = 8192 / (35 * PI)
    private var lastLeft: Double = 0.0
    private var lastRight: Double = 0.0
    private var lastHeading: Rotation2d = Rotation2d(0.0)
    private var pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
    private lateinit var drivebase: SwerveDrivebase
    private var lastTime: Long = 0

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        para = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ODOMETRY_POD_PARA).encoder
        perp = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ODOMETRY_POD_PERP).encoder
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

        /*
        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )
        imu.initialize(parameters)
         */

        para.reset()
        perp.reset()

        para.setDistancePerPulse(1/TICKS_PER_MM)
        perp.setDistancePerPulse(1/TICKS_PER_MM)

        gyro = RevIMU(hardwareMap)

        gyro.init()

        /*
        odometry = HolonomicOdometry(
            { encoderLeft.distance },
            { encoderRight.distance },
            Units.inchesToMeters(DrivebaseConstants.Measurements.TRACK_WIDTH)/1000
        )
         */

        //odometry = HolonomicOdometry( { encoderLeft.distance } )

        //odometry.updatePose(Pose2d(0.0, 0.0, Rotation2d(0.0)))

        //Pose2d(encoderLeft.distance, encoderRight.distance, Rotation2d(gyro.heading)),

        //localizer = TwoDeadWheelLocalizer(hardwareMap, imu, 1/TICKS_PER_MM)

    }

    override fun loop() {
        val currentTime = System.nanoTime()
        val currentLeft = para.distance
        val currentRight = perp.distance
        val currentHeading = gyro.rotation2d

        val deltaLeft = currentLeft - lastLeft
        val deltaRight = currentRight - lastRight
        val deltaHeading = currentHeading - lastHeading

        val dw = deltaHeading.radians
        val dx = deltaLeft
        val dy = deltaRight - (DrivebaseConstants.Measurements.CENTER_WHEEL_OFFSET * dw)

        val twist2d = Twist2d(dx, dy, dw)

        val newPose = pose2d.exp(twist2d)

        pose2d = Pose2d(newPose.translation, currentHeading)

        telemetry.addData("raw x", currentLeft)
        telemetry.addData("raw y", currentRight)
        telemetry.addData("raw heading", currentHeading)

        telemetry.addData("x", pose2d.x)
        telemetry.addData("y", pose2d.y)
        telemetry.addData("heading", pose2d.heading)
        telemetry.addData("hz ", 1E9/(currentTime-lastTime))

        /*
        telemetry.addData("perp", encoderLeft.distance)
        telemetry.addData("perp fuckup", encoderLeft.correctedVelocity - encoderLeft.rawVelocity)
        telemetry.addData("para", encoderRight.distance)
        telemetry.addData("para fuckup", encoderRight.correctedVelocity - encoderRight.rawVelocity)

         */
        //telemetry.addData("heading", imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))

        //odometry.updatePose()
        lastLeft = para.distance
        lastRight = perp.distance
        lastHeading = gyro.rotation2d
        lastTime = currentTime
        telemetry.update()
        drivebase.periodic()
    }
}