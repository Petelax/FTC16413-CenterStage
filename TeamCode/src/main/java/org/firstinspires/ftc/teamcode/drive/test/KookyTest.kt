package org.firstinspires.ftc.teamcode.drive.test

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.kooky.KookySwerveDrivetrain
import kotlin.math.PI

@TeleOp
class KookyTest: OpMode() {
    private lateinit var drive: KookySwerveDrivetrain
    override fun init() {
        drive =
            KookySwerveDrivetrain(
                hardwareMap
            )
    }

    override fun loop() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = gamepad1.left_stick_y.toDouble()
        val angle = gamepad1.right_stick_x * PI

        val pose = Pose2d(x, y, Rotation2d(angle))

    }
}