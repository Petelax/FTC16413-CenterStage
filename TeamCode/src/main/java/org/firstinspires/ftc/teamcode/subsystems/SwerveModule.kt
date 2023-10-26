package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.PI

class SwerveModule(hardwareMap: HardwareMap, driveMotorID: String, turnMotorID: String, encoderID: String, encoderOffset: Double = 0.0, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE) {
    private var drive: Motor = Motor(hardwareMap, driveMotorID)
    private var turn: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, turnMotorID)
    private var encoder: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    private var currentAngle = 0.0
    private var lastError = 0.0
    private var lastTimeStamp = 0.0
    private var desiredAngle = 0.0
    private var desiredSpeed = 0.0
    private var wheelFlipped = false
    private var state: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(0.0))
    private var maintainHeading: Boolean = false
    private var rotationController: PIDFController = PIDFController(0.0, 0.0, 0.0, 0.0)


    init {
        drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        drive.inverted = direction != DcMotorSimple.Direction.FORWARD

        turn.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
        turn.direction = DcMotorSimple.Direction.REVERSE
        encoder.init(hardwareMap, encoderID, encoderOffset)

        val pids = DrivebaseConstants.TurnPIDConstants
        rotationController.p = pids.kP
        rotationController.d = pids.kD
        rotationController.i = pids.kI

    }

    /**
     * Sets the angle of the module in degrees
     * @param setpoint Angle in degrees, counterclockwise is positive
     */
    fun setAngle(setpoint: Double) {
        val targetAngle = if (setpoint > 0) { setpoint % 360.0 } else { (setpoint % 360.0) + 360.0 }

        var diff = targetAngle - currentAngle
        if (abs(diff) > 180.0) {
            diff = diff - (sign(diff) * 360)
        }

        val flippedTargetAngle = (targetAngle + 180).mod(360.0)
        var flippedDiff = flippedTargetAngle - currentAngle
        if (abs(flippedDiff) > 180.0) {
            flippedDiff = flippedDiff - (sign(flippedDiff) * 360)
        }

        if (abs(diff) <= abs(flippedDiff)) {
            wheelFlipped = false
            desiredAngle = diff
        }
        else {
            wheelFlipped = true
            desiredAngle = flippedDiff
        }

        maintainHeading = false
        /* one thing im not satisfied with in this that i calculate the
        relative angle here, so going from 10 deg to 350 it'd set desiredAngle
        to -20. Because of my incompetence, I have been using this as the error
        in for PID for the rotation since that looks like error between my pv
        and sp. This approach means that I am unable to (or not smart enough to)
        use FTClib's PIDF controllers.
         */

    }

    /**
     * just code of me copying Kooky, please ignore
     */
    fun setKookyAngle(rotation2d: Rotation2d) {
        desiredAngle = rotation2d.degrees
    }


    /**
     * just code of me copying Kooky, please ignore
     */
    fun setKookySpeed(speed: Double) {
        if (wheelFlipped) desiredSpeed = speed * -1
        drive.set(desiredSpeed)
    }

    /**
     * just code of me copying Kooky, please ignore
     */
    fun updateKooky() {
        val pids = DrivebaseConstants.TurnPIDConstants
        val P = pids.kP
        val D = pids.kD
        val I = pids.kI
        rotationController.setPIDF(P, I, D, 0.0)
        var target: Double = desiredAngle
        val current = getModuleRotation()

        var error = normalizeRadians(target - current)
        if (abs(error) > PI / 2) {
            target = normalizeRadians(target - Math.PI)
            wheelFlipped = true
        } else {
            wheelFlipped = false
        }

        error = normalizeRadians(target - current)

        var power = Range.clip(rotationController.calculate(0.0, error), -1.0, 1.0)
        if (power.isNaN()) power = 0.0
        turn.power = power + if(abs(error) > 0.02) {0.123} else {0.0} * power.sign
    }

    /**
     * also for Kooky kopying
     */
    private fun getModuleRotation(): Double {
        return encoder.getAngle()*PI/180.0
    }

    fun update() {
        currentAngle = encoder.getAngle()
        state.angle = Rotation2d(currentAngle*(PI/180))
        state.speedMetersPerSecond = desiredSpeed*DrivebaseConstants.Measurements.MAX_VELOCITY
        if (maintainHeading) {
            turn.power = 0.0
            drive.set(0.0)
        } else {
            val currentTimeStamp = System.nanoTime() / 1E9
            val error = desiredAngle
            val period = currentTimeStamp - lastTimeStamp
            val d = (error - lastError) / period

            val speed = desiredSpeed

            drive.set(if (!wheelFlipped) { speed } else { speed*-1 })
            turn.power = DrivebaseConstants.TurnPIDConstants.kP * error + DrivebaseConstants.TurnPIDConstants.kD * d
            lastError = error
            lastTimeStamp = currentTimeStamp

        }
    }
    fun maintainHeading() {
        maintainHeading = true
    }

    fun setSwerveModuleState(state: SwerveModuleState) {
        setAngle(state.angle.degrees)
        setSpeed(state.speedMetersPerSecond/DrivebaseConstants.Measurements.MAX_VELOCITY)
    }

    fun getDriveEncoder(): Motor.Encoder {
        return drive.encoder
    }

    fun getAngle(): Double {
        return currentAngle
    }

    fun getDesiredAngle(): Double {
        return desiredAngle
    }

    fun getTurnPower(): Double {
        return turn.power
    }

    /**
     * Set drive speed of module
     * @param speed -1.0 to 1.0
     */
    fun setSpeed(speed: Double) {
        desiredSpeed = speed
    }

    fun getSpeed(): Double {
        return drive.get()
    }

}