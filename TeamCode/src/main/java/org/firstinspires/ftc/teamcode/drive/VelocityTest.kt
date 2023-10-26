package org.firstinspires.ftc.teamcode.drive

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Drivebase
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import java.io.File
import java.util.logging.Level
import java.util.logging.Logger
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.hypot

@Config
@TeleOp
class VelocityTest: OpMode() {
    private lateinit var leftEncoder: MotorEx
    private lateinit var rightEncoder: MotorEx
    private lateinit var diffOdo: DifferentialOdometry
    private lateinit var drivebase: SwerveDrivebase
    private var lastTime: Long = 0
    private var lastX = 0.0
    private var lastY = 0.0
    private var lastHead = 0.0
    private var topx = 0.0
    private var topy = 0.0
    private val etime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    private val logger = Logger.getLogger(VelocityTest::class.simpleName)

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        leftEncoder = MotorEx(hardwareMap, "motorLR")
        rightEncoder = MotorEx(hardwareMap, "motorRF")
        drivebase = SwerveDrivebase(hardwareMap)

        diffOdo = DifferentialOdometry(
            { leftEncoder.currentPosition.toDouble() },
            { rightEncoder.currentPosition.toDouble() },
            DrivebaseConstants.Measurements.TRACK_WIDTH
        )

        etime.reset()
    }

    override fun loop() {
        val loop = (System.nanoTime() - lastTime) / 1e9
        val heading = drivebase.getHeading()
        diffOdo.updatePosition( leftEncoder.currentPosition.toDouble(), rightEncoder.currentPosition.toDouble() )
        telemetry.addData("x", diffOdo.pose.x)
        telemetry.addData("y", diffOdo.pose.y)
        telemetry.addData("heading", heading*PI/180)

        val xspeed = (lastX - diffOdo.pose.x)*DrivebaseConstants.Measurements.TICKS_PER_REV / loop
        val yspeed = (lastY - diffOdo.pose.y)*DrivebaseConstants.Measurements.TICKS_PER_REV / loop
        val angularVelo = (lastHead - heading)/loop
        telemetry.addData("xspeed", xspeed)
        telemetry.addData("yspeed", yspeed)
        telemetry.addData("angularVelo", drivebase.getAngularVelocity().xRotationRate)

        val time = etime.milliseconds()
        val msg = "$time, $heading"

        logger.log(Level.INFO, msg)

        if (gamepad1.a) {
            drivebase.translate(0.0, 1.0)
        }

        if (gamepad1.b) {
            drivebase.turnInplace(1.0)
        } else {
            drivebase.turnInplace(0.0)
        }

        //robot.translate(angle, magnitude)
        //robot.turnInplace(gamepad1.left_stick_x.toDouble())

        if (xspeed>topx) {topx=xspeed}
        if (yspeed>topy) {topy=yspeed}

        lastTime = System.nanoTime()
        lastX = diffOdo.pose.x
        lastY = diffOdo.pose.y
        lastHead = heading
    }
}