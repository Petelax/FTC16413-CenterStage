package org.firstinspires.ftc.teamcode.drive

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.Robot
import kotlin.math.PI

class AutoTest: LinearOpMode() {
    private lateinit var robot: Robot
    override fun runOpMode() {
        robot = Robot(this)
        robot.setPose(Pose2d(12.0, 58.0, Rotation2d(-PI/2)))
        val headingController = PIDFController(0.01, 0.0, 0.005, 0.0)
        val xController = PIDFController(0.01, 0.0, 0.005, 0.0)
        val yController = PIDFController(0.01, 0.0, 0.005, 0.0)

        waitForStart()

        var pose = robot.getPose()
        val targetPose = Pose2d(12.0, 36.0, Rotation2d(-PI/2))
        headingController.setPoint = targetPose.heading
        xController.setPoint = targetPose.x
        yController.setPoint = targetPose.y

        while (pose != targetPose) {
            pose = robot.getPose()
            val vx = DrivebaseConstants.Measurements.MAX_VELOCITY * xController.calculate(pose.x)
            val vy = DrivebaseConstants.Measurements.MAX_VELOCITY * xController.calculate(pose.y)
            val omega = DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY * headingController.calculate(pose.heading)
            robot.kinematicsDrive(ChassisSpeeds(vx, vy, omega))
            robot.periodic()
        }

        robot.kinematicsDrive(ChassisSpeeds(0.0, 0.0, 0.0))

    }

}