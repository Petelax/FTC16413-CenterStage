package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.wpilib.SlewRateLimiter
import org.firstinspires.ftc.teamcode.wpilib.WpilibPIDController
import kotlin.math.abs

class SwerveModule(hardwareMap: HardwareMap, driveMotorID: String, turnMotorID: String, encoderID: String, encoderOffset: Double = 0.0, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE) {
    private var drive: Motor
    private var turn: CRServoImplEx
    private var encoder: AbsoluteAnalogEncoder
    private var wheelFlipped = false
    private var currentState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    private var desiredState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    private var maintainHeading: Boolean = false
    private var rotationController: WpilibPIDController =
        WpilibPIDController(0.0, 0.0, 0.0)
    private var slewRateLimiter: SlewRateLimiter

    init {
        drive = Motor(hardwareMap, driveMotorID)
        turn = hardwareMap.get(CRServoImplEx::class.java, turnMotorID)
        encoder = AbsoluteAnalogEncoder(hardwareMap, encoderID, encoderOffset)

        drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        drive.inverted = direction != DcMotorSimple.Direction.FORWARD

        turn.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
        turn.direction = DcMotorSimple.Direction.REVERSE

        val pids = DrivebaseConstants.TurnPIDConstants
        rotationController.p = pids.kP
        rotationController.d = pids.kD
        rotationController.i = pids.kI

        currentState.angle = Rotation2d(encoder.getRadians())

        rotationController.enableContinuousInput(0.0, 360.0)

        slewRateLimiter = SlewRateLimiter(DrivebaseConstants.SlewRates.POSITIVE_RATE_LIMIT, DrivebaseConstants.SlewRates.NEGATIVE_RATE_LIMIT, 0.0)

    }

    /**
     * Sets the angle of the module in degrees
     * @param setpoint Angle in radians, counterclockwise is positive
     */
    fun setAngle(setpoint: Double) {
        desiredState.angle = Rotation2d(setpoint)
    }

    fun update() {
        currentState.angle = Rotation2d(encoder.getRadians())
        //currentState.speedMetersPerSecond = drive.get()*DrivebaseConstants.Measurements.MAX_VELOCITY
        currentState.speedMetersPerSecond = drive.get()*DrivebaseConstants.Measurements.MAX_VELOCITY

        desiredState = SwerveModuleState.optimize(desiredState, currentState.angle)

        if (!maintainHeading) {maintainHeading = abs(desiredState.speedMetersPerSecond) < 0.001}

        if (maintainHeading) {
            turn.power = 0.0
            drive.set(0.0)
        } else {
            turn.power = rotationController.calculate(currentState.angle.degrees, desiredState.angle.degrees)
            drive.set(
                slewRateLimiter.calculate(desiredState.speedMetersPerSecond/DrivebaseConstants.Measurements.MAX_VELOCITY)
                //desiredState.speedMetersPerSecond/DrivebaseConstants.Measurements.MAX_VELOCITY
            )

        }
    }
    fun maintainHeading() {
        maintainHeading = true
    }

    fun setSwerveModuleState(state: SwerveModuleState) {
        this.desiredState = state
        maintainHeading = false
    }

    fun getDriveEncoder(): Motor.Encoder {
        return drive.encoder
    }

    fun getAngle(): Double {
        return currentState.angle.degrees
    }

    fun getDesiredAngle(): Double {
        return desiredState.angle.degrees
    }

    fun getTurnPower(): Double {
        return turn.power
    }

    /**
     * Set drive speed of module
     * @param speed -1.0 to 1.0
     */
    fun setSpeed(speed: Double) {
        this.desiredState.speedMetersPerSecond = speed*DrivebaseConstants.Measurements.MAX_VELOCITY
    }

    fun setSpeedMetersPerSecond(speed: Double) {
        this.desiredState.speedMetersPerSecond = speed
    }

    fun getSpeed(): Double {
        return currentState.speedMetersPerSecond
    }

    fun setPID(kP: Double, kI: Double, kD: Double) {
        rotationController.setPID(kP, kI, kD)
    }

}