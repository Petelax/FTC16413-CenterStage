package org.firstinspires.ftc.teamcode.drive.commands

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.geometry.Vector2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class FieldCentricDrive(drivebase: SwerveDrivebase, /*gamepad: GamepadEx*/
                        strafeSpeed: DoubleSupplier,
                        forwardSpeed: DoubleSupplier,
                        turnSpeed: DoubleSupplier,
                        halfSpeed: BooleanSupplier,
                        fieldCentric: BooleanSupplier

): CommandBase() {
    private var drivebase: SwerveDrivebase
    private var strafeSpeed: DoubleSupplier
    private var forwardSpeed: DoubleSupplier
    private var turnSpeed: DoubleSupplier
    private var halfSpeed: BooleanSupplier
    private var fieldCentric: BooleanSupplier

    /*
    private var gamepad: GamepadEx
     */

    init {
        this.drivebase = drivebase
        this.strafeSpeed = strafeSpeed
        this.forwardSpeed = forwardSpeed
        this.turnSpeed = turnSpeed
        this.halfSpeed = halfSpeed
        this.fieldCentric = fieldCentric
        //this.gamepad = gamepad

        addRequirements(drivebase)
    }

    override fun initialize() {
    }

    override fun execute() {
        val heading = drivebase.getHeading()
        /*
        val strafeSpeed = gamepad.leftX
        val forwardSpeed = gamepad.leftY
        val turnSpeed = gamepad.rightX
         */



        val multiplier = if (halfSpeed.asBoolean) {
            0.5
        } else {
            1.0
        }

        val vx = MathUtil.clamp(strafeSpeed.asDouble * multiplier, -1.0, 1.0)
        val vy = MathUtil.clamp(forwardSpeed.asDouble * multiplier, -1.0, 1.0)
        var omega = MathUtil.clamp(turnSpeed.asDouble * multiplier, -1.0, 1.0)

        if (vx < 0.001 && vy < 0.001) {
            omega *= 2
        }

        val input = if (fieldCentric.asBoolean) {
            Vector2d(vx, vy).rotateBy(-heading)
        } else {
            Vector2d(vx, vy)
        }

        val speeds = ChassisSpeeds(
            input.x* DrivebaseConstants.Measurements.MAX_VELOCITY,
            input.y* DrivebaseConstants.Measurements.MAX_VELOCITY,
            omega* DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY)


        drivebase.kinematicsDrive(speeds)
    }

    override fun end(interrupted: Boolean) {
        drivebase.maintainHeading()
    }

    override fun isFinished(): Boolean {
        return false
    }

}