package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

class Wrist(hardwareMap: HardwareMap): SubsystemBase() {
    //private var wrist: SimpleServo
    private var wrist: ServoImplEx

    init {
        //wrist = SimpleServo(hardwareMap, DrivebaseConstants.DeviceIDs.WRIST, 0.0, 90.0)
        wrist = hardwareMap.get(ServoImplEx::class.java, DrivebaseConstants.DeviceIDs.WRIST)
        wrist.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        wrist.position = DrivebaseConstants.WristPositions.BOTTOM

    }

    override fun periodic() {

    }

    fun set(magnitude: Double) {
        wrist.position = magnitude
    }
}