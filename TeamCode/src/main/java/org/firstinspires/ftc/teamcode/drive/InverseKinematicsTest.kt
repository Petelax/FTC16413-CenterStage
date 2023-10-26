package org.firstinspires.ftc.teamcode.drive

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.hypot

/**
 * Actual main teleop code i've been testing
 */
@TeleOp
class InverseKinematicsTest: OpMode() {
    private lateinit var robot: Robot

    override fun init() {
        robot = Robot(this)

    }

    override fun loop() {
        val x = -1.0*gamepad1.left_stick_x.toDouble()
        val y = -1.0*gamepad1.left_stick_y.toDouble()
        //val angle: Double = (atan2(y, x) * 180.0 / PI) - 90.0
        val magnitude: Double = hypot(x, y)
        val turn = gamepad1.right_stick_x.toDouble()
        //robot.translate(angle, magnitude)
        //robot.turnInplace(gamepad1.left_stick_x.toDouble())

        // This is probably bad, especially since i got these values by driving
        // and looking at the slope of the d-t graph.
        val speeds = ChassisSpeeds(
            x*DrivebaseConstants.Measurements.MAX_VELOCITY,
            y*DrivebaseConstants.Measurements.MAX_VELOCITY,
            turn*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY)

        if (magnitude<0.001 && abs(turn)<0.001) {
            robot.maintainHeading()
        } else if (magnitude<0.001 && abs(turn)>0.01) {
            robot.turnInplace(turn)
        } else {
            robot.kinematicsDrive(speeds)
        }

        telemetry.addData("x", "%.2f".format(x))
        telemetry.addData("y", "%.2f".format(y))
        robot.status()
        robot.periodic()
    }
}