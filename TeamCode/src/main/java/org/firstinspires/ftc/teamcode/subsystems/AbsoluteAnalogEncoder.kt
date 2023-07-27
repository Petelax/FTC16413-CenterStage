package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

class AbsoluteAnalogEncoder() {
    private lateinit var encoder: AnalogInput
    private val range: Double = 3.39

    fun init(hardwareMap: HardwareMap, encoderID: String) {
        encoder = hardwareMap.get(AnalogInput::class.java, encoderID)
    }

    fun getAngle(): Double {
        return (360 * (encoder.voltage / range)) % 360
    }

    fun getVoltage(): Double {
        return encoder.voltage
    }

}