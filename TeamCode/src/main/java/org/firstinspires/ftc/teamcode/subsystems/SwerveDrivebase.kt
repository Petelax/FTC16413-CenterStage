package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.hardware.GyroEx
import com.arcrobotics.ftclib.hardware.RevIMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class SwerveDrivebase(hardwareMap: HardwareMap): Drivebase() {
    private lateinit var swerveLF: SwerveModule
    private lateinit var swerveRF: SwerveModule
    private lateinit var swerveLR: SwerveModule
    private lateinit var swerveRR: SwerveModule
    private lateinit var gyro: RevIMU
    var enabled = true
        set(value) {
            field = value
            swerveLF.enabled = value
            swerveRF.enabled = value
            swerveLR.enabled = value
            swerveRR.enabled = value

        }

    init {
        val IDs = DrivebaseConstants.DeviceIDs

        gyro = RevIMU(hardwareMap)
        gyro.init()

        swerveLF = SwerveModule(hardwareMap, IDs.LF_DRIVE_MOTOR, IDs.LF_TURN_MOTOR, IDs.LF_ENCODER)
        swerveRF = SwerveModule(hardwareMap, IDs.RF_DRIVE_MOTOR, IDs.RF_TURN_MOTOR, IDs.RF_ENCODER)
        swerveLR = SwerveModule(hardwareMap, IDs.LR_DRIVE_MOTOR, IDs.LR_TURN_MOTOR, IDs.LR_ENCODER)
        swerveRR = SwerveModule(hardwareMap, IDs.RR_DRIVE_MOTOR, IDs.RR_TURN_MOTOR, IDs.RR_ENCODER)

    }

    /**
     * Translate drivebase
     */
    fun translate(speed: Double, angle: Double) {
        swerveLF.override = false
        swerveRF.override = false
        swerveLR.override = false
        swerveRR.override = false

        swerveLF.setAngle(angle)
        swerveRF.setAngle(angle)
        swerveLR.setAngle(angle)
        swerveRR.setAngle(angle)

        swerveLF.setSpeed(speed)
        swerveRF.setSpeed(speed)
        swerveLR.setSpeed(speed)
        swerveRR.setSpeed(speed)
    }

    fun spin(speed: Double) {
        swerveLF.override = true
        swerveRF.override = true
        swerveLR.override = true
        swerveRR.override = true

        swerveLF.setAngle(0.0)
        swerveRF.setAngle(0.0)
        swerveLR.setAngle(0.0)
        swerveRR.setAngle(0.0)

        swerveLF.setOverrideTurnSpeed(speed)
        swerveRF.setOverrideTurnSpeed(speed)
        swerveLR.setOverrideTurnSpeed(speed)
        swerveRR.setOverrideTurnSpeed(speed)

        swerveLF.setSpeed(0.0)
        swerveRF.setSpeed(0.0)
        swerveLR.setSpeed(0.0)
        swerveRR.setSpeed(0.0)
    }

    fun spinAbsolute(angle: Double) {
        swerveLF.setAbsoluteAngle(angle)
        swerveRF.setAbsoluteAngle(angle)
        swerveLR.setAbsoluteAngle(angle)
        swerveRR.setAbsoluteAngle(angle)

        swerveLF.setSpeed(0.0)
        swerveRF.setSpeed(0.0)
        swerveLR.setSpeed(0.0)
        swerveRR.setSpeed(0.0)
    }

    fun reset() {
        swerveLF.setAngle(0.0)
        swerveRF.setAngle(0.0)
        swerveLR.setAngle(0.0)
        swerveRR.setAngle(0.0)

        swerveLF.setSpeed(0.0)
        swerveRF.setSpeed(0.0)
        swerveLR.setSpeed(0.0)
        swerveRR.setSpeed(0.0)
    }

    fun status(): String {
        val LF = "angle: " + swerveLF.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveLF.getSpeed().toString() + ", turnPower: " + swerveLF.getTurnPower().toString()
        val RF = "angle: " + swerveRF.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveRF.getSpeed().toString() + ", turnPower: " + swerveRF.getTurnPower().toString()
        val LR = "angle: " + swerveLR.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveLR.getSpeed().toString() + ", turnPower: " + swerveLR.getTurnPower().toString()
        val RR = "angle: " + swerveRR.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveRR.getSpeed().toString() + ", turnPower: " + swerveRR.getTurnPower().toString()

        val status = LF + "\n" + RF + "\n" + LR + "\n" + RR

        return status
    }

    fun getHeading(): Double {
        return gyro.heading
    }

    /**
     * Update drivebase motors
     */
    fun update() {
        swerveLF.update()
        swerveRF.update()
        swerveLR.update()
        swerveRR.update()
    }
}