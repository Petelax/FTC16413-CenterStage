package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config

@Config
class DrivebaseConstants {
    object DeviceIDs {
        const val LF_DRIVE_MOTOR = "motorLF"
        const val LF_TURN_MOTOR = "servoLF"
        const val LF_ENCODER = "encoderLF"

        const val RF_DRIVE_MOTOR = "motorRF"
        const val RF_TURN_MOTOR = "servoRF"
        const val RF_ENCODER = "encoderRF"

        const val LR_DRIVE_MOTOR = "motorLR"
        const val LR_TURN_MOTOR = "servoLR"
        const val LR_ENCODER = "encoderLR"

        const val RR_DRIVE_MOTOR = "motorRR"
        const val RR_TURN_MOTOR = "servoRR"
        const val RR_ENCODER = "encoderRR"
    }
    object TurnPIDConstants {
        const val kP = 0.003
        const val kI = 0.0
        const val kD = 0.0
    }
}