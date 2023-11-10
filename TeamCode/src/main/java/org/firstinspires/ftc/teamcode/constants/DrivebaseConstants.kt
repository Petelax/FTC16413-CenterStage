package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.geometry.Translation2d

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

        const val LEFT_ODOMETRY_POD = LR_DRIVE_MOTOR
        const val RIGHT_ODOMETRY_POD = RF_DRIVE_MOTOR
    }
    object TurnPIDConstants {
        const val kP = 0.012
        const val kI = 0.0
        const val kD = 0.00005
    }

    /**
     * measured in inches
     * (im lying some are meters or mm or god knows what)
     */
    object Measurements {
        const val PI = 3.1415926535897932384626433832795028841971693993751058209
        const val TRACK_WIDTH = 10.50
        const val WHEEL_BASE = 10.50
        const val WHEEL_RADIUS = 2.835/2.0
        const val TICKS_PER_REV = 8192
        const val TICKS_TO_INCHES = (PI*2.0*WHEEL_RADIUS) / TICKS_PER_REV
        const val MAX_VELOCITY = 3.8 /* meters per second */
        const val MAX_ANGULAR_VELOCITY = 16.0
        private const val k = TRACK_WIDTH*0.0254/2
        private const val j = WHEEL_BASE*0.0254/2

        val LF_POS = Translation2d(k/2, j/2)
        val RF_POS = Translation2d(k/2, -j/2)
        val LR_POS = Translation2d(-k/2, j/2)
        val RR_POS = Translation2d(-k/2, -j/2)

        const val LF_OFFSET = -1.175
        const val RF_OFFSET = -0.23
        const val LR_OFFSET = -0.98
        const val RR_OFFSET = -1.85


    }
}