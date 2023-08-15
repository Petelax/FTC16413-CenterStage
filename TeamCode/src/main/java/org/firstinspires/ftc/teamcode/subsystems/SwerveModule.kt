package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import kotlin.math.abs
import kotlin.math.sign

class SwerveModule(hardwareMap: HardwareMap, driveMotorID: String, turnMotorID: String, encoderID: String) {
    private var drive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, driveMotorID)
    private var turn: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, turnMotorID)
    private var encoder: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    private var currentAngle = 0.0
    private var desiredAngle = 0.0
    private var wheelFlipped = false


    init {
        drive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turn.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
        encoder.init(hardwareMap, encoderID)
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

        val flippedTargetAngle = (targetAngle + 180) % 360
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

        currentAngle = encoder.getAngle()
        val error = desiredAngle
        val power = error * DrivebaseConstants.TurnPIDConstants.kP

        turn.power = power

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
        drive.power = if (!wheelFlipped) { speed } else { speed*-1 }
    }

    fun getSpeed(): Double {
        return drive.power
    }

}