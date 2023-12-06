package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class Claw(hardwareMap: HardwareMap): SubsystemBase() {
    private var clawLeft: SimpleServo
    private var clawRight: SimpleServo
    init {
        clawLeft = SimpleServo(hardwareMap, DrivebaseConstants.DeviceIDs.CLAW_LEFT, -10.0, 10.0)
        clawRight = SimpleServo(hardwareMap, DrivebaseConstants.DeviceIDs.CLAW_RIGHT, -10.0, 10.0)

        clawLeft.position = 0.5
        clawRight.position = 0.5
    }

    override fun periodic() {

    }

    fun setGripper(magnitude: Double) {
        clawLeft.position = magnitude
        clawRight.position = 1.0-magnitude
    }

    fun setRight(magnitude: Double) {
        clawRight.position = 1.0-magnitude
    }

    fun setLeft(magnitude: Double) {
        clawLeft.position = magnitude
    }

    fun getGrabber(): Double {
        return clawLeft.position
    }

}