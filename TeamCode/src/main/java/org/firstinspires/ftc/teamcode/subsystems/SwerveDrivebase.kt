package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.hardware.RevIMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class SwerveDrivebase(hardwareMap: HardwareMap): Drivebase() {
    private var swerveLF: SwerveModule
    private var swerveRF: SwerveModule
    private var swerveLR: SwerveModule
    private var swerveRR: SwerveModule
    private var gyro: RevIMU

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
    fun translate(angle: Double, speed: Double) {
        swerveLF.setAngle(angle)
        swerveRF.setAngle(angle)
        swerveLR.setAngle(angle)
        swerveRR.setAngle(angle)

        swerveLF.setSpeed(speed)
        swerveRF.setSpeed(speed)
        swerveLR.setSpeed(speed)
        swerveRR.setSpeed(speed)
    }

    fun turnInplace(speed: Double) {
        swerveLF.setAngle(45.0)
        swerveRF.setAngle(135.0)
        swerveLR.setAngle(-45.0)
        swerveRR.setAngle(-135.0)

        swerveLF.setSpeed(speed)
        swerveRF.setSpeed(speed)
        swerveLR.setSpeed(speed)
        swerveRR.setSpeed(speed)

    }


    fun status(): String {
        val lf = "angle: " + swerveLF.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveLF.getSpeed().toString() + ", turnPower: " + swerveLF.getTurnPower().toString()
        val rf = "angle: " + swerveRF.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveRF.getSpeed().toString() + ", turnPower: " + swerveRF.getTurnPower().toString()
        val lr = "angle: " + swerveLR.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveLR.getSpeed().toString() + ", turnPower: " + swerveLR.getTurnPower().toString()
        val rr = "angle: " + swerveRR.getAngle().toString() + ", desiredAngle: " + swerveLF.getDesiredAngle().toString() + ", speed: " + swerveRR.getSpeed().toString() + ", turnPower: " + swerveRR.getTurnPower().toString()

        val status = lf + "\n" + rf + "\n" + lr + "\n" + rr

        return status
    }

    fun getHeading(): Double {
        return gyro.heading
    }

}