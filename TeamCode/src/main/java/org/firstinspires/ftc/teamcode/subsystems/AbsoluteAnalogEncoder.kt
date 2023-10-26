package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

class AbsoluteAnalogEncoder() {
    private lateinit var encoder: AnalogInput
    private var range: Double = 3.28
    private val relativeAngle = 0.0
    private var offset: Double = 0.0

    fun init(hardwareMap: HardwareMap, encoderID: String, offset: Double = 0.0) {
        encoder = hardwareMap.get(AnalogInput::class.java, encoderID)
        this.offset = offset
    }

    fun getAngle(): Double {
        return (360 * ((encoder.voltage + offset).mod(range) / range)) % 360
    }

    fun getRelativeAngle(): Double {

        return 0.0
    }

    fun periodic() {

    }

    fun getVoltage(): Double {
        return (encoder.voltage + offset).mod(range)
    }

    fun setRange(n: Double) {
        range = n
    }

    fun setOffset(n: Double) {
        offset = n
    }

}