package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class Intake(hardwareMap: HardwareMap): SubsystemBase() {
    private var motor: Motor
    init {
        motor = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.INTAKE, Motor.GoBILDA.RPM_1150)

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
    }

    fun set(speed: Double) {
        motor.set(speed)
    }

    override fun periodic() {

    }
}