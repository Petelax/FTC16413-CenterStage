package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import kotlin.math.abs
import kotlin.math.sign

class SwerveModule(hardwareMap: HardwareMap, driveMotorID: String, turnMotorID: String, encoderID: String) {
    private var drive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, driveMotorID)
    private var turn: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, turnMotorID)
    private var encoder: AbsoluteAnalogEncoder = AbsoluteAnalogEncoder()
    private var drivePower = 0.0
    private var turnPower = 0.0
    private var calcTurnPower = 0.0
    private var currentAngle = 0.0
    private var desiredAngle = 0.0
    private var wheelFlipped = false
    private var voltage = 0.0
    var enabled = true
    var override = false

    private var turnController: PIDController

    init {
        drive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turn.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
        encoder.init(hardwareMap, encoderID)
        turnController = PIDController(DrivebaseConstants.TurnPIDConstants.kP, DrivebaseConstants.TurnPIDConstants.kI, DrivebaseConstants.TurnPIDConstants.kD, desiredAngle, currentAngle)
    }

    /**
     * Sets the angle of the module in degrees
     * @param setpoint Angle in degrees, counterclockwise is positive
     */
    fun setAngle(setpoint: Double) {
        val setpointAngle = closestAngle(currentAngle, setpoint)
        val setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0)

        if (abs(setpointAngle) <= abs(setpointAngleFlipped)) {
            wheelFlipped = false
            desiredAngle = setpointAngle
        }
        else {
            wheelFlipped = true
            desiredAngle = setpointAngleFlipped
        }

    }

    fun setAbsoluteAngle(setpoint: Double) {
        desiredAngle = setpoint
    }

    fun getAngle(): Double {
        return currentAngle
    }

    fun getDesiredAngle(): Double {
        return desiredAngle
    }

    fun getTurnPower(): Double {
        return turnPower
    }

    /**
     * Set drive speed of module
     * @param speed -1.0 to 1.0
     */
    fun setSpeed(speed: Double) {
        drivePower = if (!wheelFlipped) { speed } else { speed*-1 }
    }

    fun getSpeed(): Double {
        return drivePower
    }

    /**
     * Update the drive motor and turn motor of the module
     */
    fun update() {
        currentAngle = encoder.getAngle()
        voltage = encoder.getVoltage()
        if (!override) {
            //turnPower = turnController.calculate(currentAngle, (desiredAngle + currentAngle))
            calcTurnPower = turnController.calculate(currentAngle, (desiredAngle + currentAngle))
            turnPower = (desiredAngle - currentAngle) * DrivebaseConstants.TurnPIDConstants.kP
        }
        else {
            calcTurnPower = turnController.calculate(currentAngle, (desiredAngle + currentAngle))
        }

        if (enabled) {
            drive.power = drivePower
            if (abs(currentAngle - desiredAngle) > 0.5) {
                turn.power = truncate(turnPower, -0.5, 0.5)
            }
        }
    }

    fun setOverrideTurnSpeed(speed: Double) {
        turnPower = speed
    }

    private fun truncate(n: Double, min: Double, max: Double): Double {
        var ans = n
        if (n>max) {ans=max}
        else if (n<min) {ans=min}
        return ans
    }

    /**
     * Calculate the closest angle between a and b
     * @param a Current angle in degrees
     * @param b Desired angle in degrees
     * @return Closest angle in degrees
     */
    fun closestAngle(a: Double, b: Double): Double {
        var dir: Double = (b % 360.0) - (a % 360.0)
        if (abs(dir) > 180.0) {
            dir = dir - (sign(dir) * 360)
        }
        return dir
    }

}