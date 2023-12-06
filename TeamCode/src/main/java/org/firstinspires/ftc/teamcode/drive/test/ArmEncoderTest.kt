package org.firstinspires.ftc.teamcode.drive.test

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants

@TeleOp(group = "test")
class ArmEncoderTest: OpMode() {
    private lateinit var motor: Motor
    override fun init() {
        motor = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ARM, Motor.GoBILDA.RPM_43)

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)

    }

    override fun loop() {
        telemetry.addData("pos", motor.currentPosition)
        telemetry.update()
    }

}