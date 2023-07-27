package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PwmControl

@TeleOp
class ServoTest: OpMode() {
    private lateinit var servoLF: CRServoImplEx
    private lateinit var servoRF: CRServoImplEx
    private lateinit var servoLR: CRServoImplEx
    private lateinit var servoRR: CRServoImplEx

    private lateinit var motorLF: DcMotorEx
    private lateinit var motorRF: DcMotorEx
    private lateinit var motorLR: DcMotorEx
    private lateinit var motorRR: DcMotorEx

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        servoLF = hardwareMap.get(CRServoImplEx::class.java, "servoLF")
        servoRF = hardwareMap.get(CRServoImplEx::class.java, "servoRF")
        servoLR = hardwareMap.get(CRServoImplEx::class.java, "servoLR")
        servoRR = hardwareMap.get(CRServoImplEx::class.java, "servoRR")

        motorLF = hardwareMap.get(DcMotorEx::class.java, "motorLF")
        motorRF = hardwareMap.get(DcMotorEx::class.java, "motorRF")
        motorLR = hardwareMap.get(DcMotorEx::class.java, "motorLR")
        motorRR = hardwareMap.get(DcMotorEx::class.java, "motorRR")

        motorLF.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motorRF.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motorLR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motorRR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        servoLF.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoRF.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoLR.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servoRR.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        // LF -> LR
        // RF -> LF
        // RR -> RF
        // LR -> RR

    }

    override fun loop() {
        val power: Double = gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", power)

        if (gamepad1.x) { servoLF.power = power; telemetry.addLine("servoLF") } else { servoLF.power = 0.0 }
        if (gamepad1.y) { servoRF.power = power; telemetry.addLine("servoRF") } else { servoLF.power = 0.0 }
        if (gamepad1.a) { servoLR.power = power; telemetry.addLine("servoLR") } else { servoLF.power = 0.0 }
        if (gamepad1.b) { servoRR.power = power; telemetry.addLine("servoRR") } else { servoLF.power = 0.0 }

        if (gamepad1.dpad_up) { motorLF.power = power; telemetry.addLine("motorLF") } else { motorLF.power = 0.0 }
        if (gamepad1.dpad_left) { motorRF.power = power; telemetry.addLine("motorRF") } else { motorLF.power = 0.0 }
        if (gamepad1.dpad_down) { motorLR.power = power; telemetry.addLine("motorLR") } else { motorLF.power = 0.0 }
        if (gamepad1.dpad_right) { motorRR.power = power; telemetry.addLine("motorRR") } else { motorLF.power = 0.0 }


    }
}