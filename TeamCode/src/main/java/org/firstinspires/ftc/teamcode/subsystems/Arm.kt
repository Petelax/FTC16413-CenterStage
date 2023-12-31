package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class Arm(hardwareMap: HardwareMap): SubsystemBase() {
    private var motor: Motor
    private var speed = 0.0
    init {
        motor = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ARM, Motor.GoBILDA.RPM_43)

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        motor.stopAndResetEncoder()

    }

    fun set(speed: Double) {
        this.speed = speed
    }

    fun get(): Double {
        return speed
    }

    override fun periodic() {
        motor.set(speed)
    }

    fun getCurrentPosition(): Double {
        return motor.currentPosition.toDouble()
    }
}