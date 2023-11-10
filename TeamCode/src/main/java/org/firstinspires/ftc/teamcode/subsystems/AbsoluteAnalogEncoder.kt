package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI

class AbsoluteAnalogEncoder(hardwareMap: HardwareMap, encoderID: String, offset: Double = 0.0) {
    private lateinit var encoder: AnalogInput
    private var range: Double = 3.28
    private val relativeAngle = 0.0
    private var offset: Double = 0.0

    init {
        encoder = hardwareMap.get(AnalogInput::class.java, encoderID)
        this.offset = offset
    }

    fun getDegrees(): Double {
        return ((encoder.voltage + offset) / range) * 360.0
    }

    fun getRadians(): Double {
        return ((encoder.voltage + offset) / range) * 2.0*PI
    }

    fun getCurrentPosition(): Double {
        return getDegrees()
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