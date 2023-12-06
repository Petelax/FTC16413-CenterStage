package org.firstinspires.ftc.teamcode.testing

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class Intake(hardwareMap: HardwareMap): SubsystemBase() {
    private var motor: Motor
    private var speed: Double = 0.0

    init {
        this.motor = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.INTAKE, Motor.GoBILDA.RPM_1150)

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)

    }

    override fun periodic() {
        motor.set(speed)
    }

    fun set(speed: Double) {
        this.speed = speed
    }

}