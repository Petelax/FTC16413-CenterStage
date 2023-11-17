package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.wpilib.MathUtil
import org.firstinspires.ftc.teamcode.wpilib.WpilibPIDController

class Elevator(hardwareMap: HardwareMap, mode: Mode) {
    private var motorLeft: Motor
    private var motorRight: Motor

    private var elevator: MotorGroup
    private var feedforward: ElevatorFeedforward
    private var pid: WpilibPIDController

    private var speed: Double = 0.0
    private var currentPosition: Double = 0.0
    var setpoint = 0.0

    var mode: Mode


    init {
        this.mode = mode
        val pids = DrivebaseConstants.ElevatorPIDConstants
        motorLeft = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ELEVATOR_LEFT, Motor.GoBILDA.RPM_435)
        motorRight = Motor(hardwareMap, DrivebaseConstants.DeviceIDs.ELEVATOR_RIGHT, Motor.GoBILDA.RPM_435)

        motorLeft.stopAndResetEncoder()
        motorRight.stopAndResetEncoder()

        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        motorRight.inverted = true

        elevator = MotorGroup(motorLeft, motorRight)
        feedforward = ElevatorFeedforward(pids.kS, pids.kG, pids.kV, pids.kA)
        pid = WpilibPIDController(pids.kP, pids.kI, pids.kD)


    }

    enum class Mode {
        PID, RAW, IDLE
    }

    /*
    120mm per rotation
    696mm total travel
     */
    fun ticksToInches(ticks: Double): Double {
        return ticks / 5.8
    }

    fun set(speed: Double) {
        this.speed = speed
    }

    fun get(): Double {
        return speed
    }

    fun getCurrentPosition(): Double {
        return currentPosition
    }

    fun getAll(): List<Int> {
        return listOf(motorLeft.currentPosition, motorRight.currentPosition)
    }

    fun update() {
        currentPosition = (motorLeft.currentPosition + motorRight.currentPosition).toDouble() / 2

        speed = when(mode) {
            Mode.RAW -> { speed }
            Mode.PID -> {
                if (MathUtil.isNear(setpoint, currentPosition, 10.0)) {
                    0.0
                } else {
                    pid.calculate(currentPosition, setpoint)
                }
            }
            Mode.IDLE -> { 0.0 }
            else -> { throw Exception("idk how, but ur bad") }
        }


        if ((currentPosition < 0.0 && speed < 0.0) || (currentPosition > 2000.0 && speed > 0.0)) {
            speed = 0.0
        }

        elevator.set(speed)

    }

    companion object {
        const val TOP = 2000.0
        const val BOTTOM = 1.0
    }


}